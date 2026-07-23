// eth_mac_tx.v — передатчик Ethernet-кадра через MII (100 Мбит: 4 бита @ 25 МГц).
//
// Принимает БАЙТЫ кадра уровня L2 (DstMAC, SrcMAC, EtherType, Payload) по
// потоковому интерфейсу s_valid/s_ready/s_last, буферизует кадр целиком, затем
// выдаёт его в MII в таком порядке:
//
//   Preamble(7×0x55)  SFD(0xD5)  [кадр, дополненный нулями до MIN_LEN]  FCS(4)  IFG
//
// FCS (CRC-32) считается по DstMAC..Payload с учётом паддинга, инвертируется и
// передаётся младшим байтом вперёд. В MII каждый байт уходит младшим ниблом
// вперёд, по 4 бита за такт TX_CLK.
//
// Почему кадр сперва буферизуется целиком: поток MII нельзя останавливать
// внутри кадра (пауза = битый кадр, underrun). Поэтому сначала «загрузка» в
// память, потом «выдача» — та же схема, что загрузка матриц в matmul.
`timescale 1ns/1ps

module eth_mac_tx #(
    parameter MAX_LEN   = 1518,   // максимум байт L2-кадра без FCS
    parameter MIN_LEN   = 60,     // паддинг до 60 → минимальный кадр 64 байта с FCS
    parameter IFG_BYTES = 12      // межкадровый интервал, байт
)(
    input  wire        tx_clk,    // 25 МГц (MII TX_CLK от PHY на 100 Мбит)
    input  wire        rst,       // синхронный, активный высокий

    // вход: байты кадра DstMAC..Payload по порядку
    input  wire        s_valid,
    output wire        s_ready,
    input  wire [7:0]  s_data,
    input  wire        s_last,    // 1 на последнем байте кадра

    // выход MII к PHY
    output reg  [3:0]  txd,       // TXD[3:0]
    output reg         tx_en,     // TX_EN
    output wire        busy
);
    localparam [2:0] S_IDLE = 3'd0,   // приём кадра в буфер
                     S_PRE  = 3'd1,   // преамбула
                     S_SFD  = 3'd2,   // Start Frame Delimiter
                     S_DATA = 3'd3,   // кадр + паддинг (+ подсчёт CRC)
                     S_FCS  = 3'd4,   // 4 байта контрольной суммы
                     S_IFG  = 3'd5;   // межкадровый интервал

    reg [2:0]  state;
    reg [7:0]  buf_mem [0:MAX_LEN-1];

    reg [31:0] len;        // сколько байт кадра принято
    reg [31:0] send_len;   // длина выдачи с учётом паддинга
    reg [31:0] idx;        // индекс текущего байта данных
    reg        nib;        // 0 = младший нибл, 1 = старший
    reg [31:0] pre_cnt;    // счётчик байт преамбулы
    reg [31:0] fcs_idx;    // 0..3 — какой байт FCS выдаём
    reg [31:0] ifg_cnt;    // счётчик ниблов паузы
    reg [31:0] crc;        // накопитель CRC (до финальной инверсии)
    reg [31:0] fcs;        // готовый FCS (уже инвертирован)

    // текущий байт данных: реальный из буфера либо 0x00 при паддинге
    wire [7:0]  data_byte = (idx < len) ? buf_mem[idx] : 8'h00;

    // комбинационный шаг CRC по текущему байту
    wire [31:0] crc_next;
    crc32 u_crc (.crc_in(crc), .data(data_byte), .crc_out(crc_next));

    // текущий байт FCS (младший первым)
    wire [7:0]  fcs_sel = fcs[8*fcs_idx +: 8];

    assign s_ready = (state == S_IDLE);
    assign busy    = (state != S_IDLE);

    always @(posedge tx_clk) begin
        if (rst) begin
            state <= S_IDLE;
            tx_en <= 1'b0;
            txd   <= 4'h0;
            len   <= 0;
            nib   <= 1'b0;
        end else begin
            case (state)
            // ---------------- приём кадра в буфер ----------------
            S_IDLE: begin
                tx_en <= 1'b0;
                txd   <= 4'h0;
                if (s_valid) begin
                    buf_mem[len] <= s_data;
                    len          <= len + 1;
                    if (s_last) begin
                        send_len <= (len + 1 < MIN_LEN) ? MIN_LEN : (len + 1);
                        idx      <= 0;
                        nib      <= 1'b0;
                        pre_cnt  <= 0;
                        crc      <= 32'hFFFF_FFFF;
                        state    <= S_PRE;
                    end
                end
            end

            // ---------------- преамбула: 7 байт 0x55 -------------
            S_PRE: begin
                tx_en <= 1'b1;
                txd   <= 4'h5;                 // 0x55 → оба нибла 0x5
                if (nib) begin
                    nib <= 1'b0;
                    if (pre_cnt == 6) state <= S_SFD;
                    else              pre_cnt <= pre_cnt + 1;
                end else nib <= 1'b1;
            end

            // ---------------- SFD: 0xD5 --------------------------
            S_SFD: begin
                txd <= nib ? 4'hD : 4'h5;      // младший нибл 0x5, затем 0xD
                if (nib) begin
                    nib   <= 1'b0;
                    state <= S_DATA;
                end else nib <= 1'b1;
            end

            // ---------------- данные + паддинг, счёт CRC ---------
            S_DATA: begin
                txd <= nib ? data_byte[7:4] : data_byte[3:0];
                if (nib) begin
                    nib <= 1'b0;
                    crc <= crc_next;           // учесть этот байт в CRC
                    if (idx + 1 == send_len) begin
                        fcs     <= ~crc_next;  // финальная инверсия
                        fcs_idx <= 0;
                        state   <= S_FCS;
                    end else idx <= idx + 1;
                end else nib <= 1'b1;
            end

            // ---------------- FCS: 4 байта, младший первым -------
            S_FCS: begin
                txd <= nib ? fcs_sel[7:4] : fcs_sel[3:0];
                if (nib) begin
                    nib <= 1'b0;
                    if (fcs_idx == 3) begin
                        ifg_cnt <= 0;
                        state   <= S_IFG;
                    end else fcs_idx <= fcs_idx + 1;
                end else nib <= 1'b1;
            end

            // ---------------- межкадровый интервал ---------------
            S_IFG: begin
                tx_en <= 1'b0;
                txd   <= 4'h0;
                if (ifg_cnt == IFG_BYTES*2 - 1) begin
                    len   <= 0;
                    state <= S_IDLE;
                end else ifg_cnt <= ifg_cnt + 1;
            end

            default: state <= S_IDLE;
            endcase
        end
    end
endmodule

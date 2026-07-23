// tb_eth_mac_tx.v — самопроверяющийся тестбенч для eth_mac_tx.
//
// Что проверяем:
//   1) движок CRC32: CRC32("123456789") == 0xCBF43926 (эталон стандарта);
//   2) собранный на выходе MII кадр: преамбула, SFD, данные+паддинг, FCS;
//      FCS считается независимо в тестбенче и сравнивается с тем, что выдал DUT.
`timescale 1ns/1ps

module tb_eth_mac_tx;
    localparam MIN_LEN = 60;

    reg        tx_clk = 1'b0;
    reg        rst    = 1'b1;
    reg        s_valid = 1'b0;
    reg        s_last  = 1'b0;
    reg  [7:0] s_data  = 8'h00;
    wire       s_ready;
    wire [3:0] txd;
    wire       tx_en;
    wire       busy;

    eth_mac_tx #(.MIN_LEN(MIN_LEN)) dut (
        .tx_clk(tx_clk), .rst(rst),
        .s_valid(s_valid), .s_ready(s_ready), .s_data(s_data), .s_last(s_last),
        .txd(txd), .tx_en(tx_en), .busy(busy)
    );

    always #20 tx_clk = ~tx_clk;   // 25 МГц → период 40 нс

    // ------- захват ниблов MII (пока tx_en) -------
    reg [3:0] nib_mem [0:4095];
    integer   ncount = 0;
    always @(posedge tx_clk) if (tx_en) begin
        nib_mem[ncount] = txd;
        ncount = ncount + 1;
    end

    // ------- эталонный шаг CRC32 (та же арифметика, что в DUT) -------
    function [31:0] crc32_step;
        input [31:0] c;
        input [7:0]  d;
        integer b;
        reg [31:0] x;
        begin
            x = c ^ {24'b0, d};
            for (b = 0; b < 8; b = b + 1)
                x = x[0] ? ((x >> 1) ^ 32'hEDB8_8320) : (x >> 1);
            crc32_step = x;
        end
    endfunction

    reg  [7:0] frame [0:63];
    integer    frame_len = 20;
    integer    send_len;
    reg [31:0] crc, exp_fcs;
    reg  [7:0] rx [0:2047];
    integer    rxn;
    reg  [7:0] eb;
    integer    i, k, errors;

    initial begin
        if ($test$plusargs("dump")) begin
            $dumpfile("tb_eth_mac_tx.vcd");
            $dumpvars(0, tb_eth_mac_tx);
        end

        errors = 0;

        // --- 1) якорь движка: CRC32("123456789") == 0xCBF43926 ---
        crc = 32'hFFFF_FFFF;
        crc = crc32_step(crc, "1"); crc = crc32_step(crc, "2"); crc = crc32_step(crc, "3");
        crc = crc32_step(crc, "4"); crc = crc32_step(crc, "5"); crc = crc32_step(crc, "6");
        crc = crc32_step(crc, "7"); crc = crc32_step(crc, "8"); crc = crc32_step(crc, "9");
        if ((crc ^ 32'hFFFF_FFFF) !== 32'hCBF4_3926) begin
            $display("FAIL: движок CRC32 неверен: %08x (ожидалось CBF43926)", crc ^ 32'hFFFF_FFFF);
            errors = errors + 1;
        end else
            $display("OK  : CRC32(\"123456789\") = 0xCBF43926");

        // --- тестовый кадр: DstMAC / SrcMAC / EtherType / payload ---
        frame[0]=8'h12; frame[1]=8'h34; frame[2]=8'h56; frame[3]=8'h78; frame[4]=8'h9A; frame[5]=8'hBC;
        frame[6]=8'hDE; frame[7]=8'hAD; frame[8]=8'hBE; frame[9]=8'hEF; frame[10]=8'h00; frame[11]=8'h01;
        frame[12]=8'h88; frame[13]=8'hB5;                      // EtherType 0x88B5 (experimental)
        frame[14]="H"; frame[15]="E"; frame[16]="L"; frame[17]="L"; frame[18]="O"; frame[19]="!";

        // --- эталонный FCS по данным + паддингу ---
        send_len = (frame_len < MIN_LEN) ? MIN_LEN : frame_len;
        crc = 32'hFFFF_FFFF;
        for (i = 0; i < send_len; i = i + 1) begin
            eb  = (i < frame_len) ? frame[i] : 8'h00;
            crc = crc32_step(crc, eb);
        end
        exp_fcs = ~crc;

        // --- прогон DUT ---
        rst = 1'b1; s_valid = 1'b0; s_last = 1'b0;
        repeat (4) @(posedge tx_clk);
        @(negedge tx_clk); rst = 1'b0;

        for (i = 0; i < frame_len; i = i + 1) begin
            s_valid = 1'b1;
            s_data  = frame[i];
            s_last  = (i == frame_len - 1);
            @(negedge tx_clk);
        end
        s_valid = 1'b0; s_last = 1'b0;

        wait (busy);        // DUT начал передачу
        wait (!busy);       // и закончил (после IFG)
        repeat (4) @(posedge tx_clk);

        // --- разбор захваченного потока ---
        rxn = ncount / 2;
        for (k = 0; k < rxn; k = k + 1)
            rx[k] = {nib_mem[2*k+1], nib_mem[2*k]};   // младший нибл первым

        for (k = 0; k < 7; k = k + 1)
            if (rx[k] !== 8'h55) begin
                $display("FAIL: преамбула[%0d] = %02x", k, rx[k]); errors = errors + 1;
            end
        if (rx[7] !== 8'hD5) begin
            $display("FAIL: SFD = %02x (ожидалось D5)", rx[7]); errors = errors + 1;
        end
        for (k = 0; k < send_len; k = k + 1) begin
            eb = (k < frame_len) ? frame[k] : 8'h00;
            if (rx[8+k] !== eb) begin
                $display("FAIL: данные[%0d] = %02x (ожидалось %02x)", k, rx[8+k], eb);
                errors = errors + 1;
            end
        end
        if (rx[8+send_len+0] !== exp_fcs[7:0])   begin $display("FAIL: FCS0"); errors = errors + 1; end
        if (rx[8+send_len+1] !== exp_fcs[15:8])  begin $display("FAIL: FCS1"); errors = errors + 1; end
        if (rx[8+send_len+2] !== exp_fcs[23:16]) begin $display("FAIL: FCS2"); errors = errors + 1; end
        if (rx[8+send_len+3] !== exp_fcs[31:24]) begin $display("FAIL: FCS3"); errors = errors + 1; end

        $display("info: ниблов=%0d, байт=%0d, send_len=%0d, FCS=%08x",
                 ncount, rxn, send_len, exp_fcs);
        if (errors == 0)
            $display("PASS: MII-кадр корректен (преамбула + SFD + данные/паддинг + FCS)");
        else
            $display("FAIL: ошибок = %0d", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("FAIL: timeout");
        $finish;
    end
endmodule

// crc32.v — комбинационный шаг Ethernet FCS (CRC-32/ISO-HDLC) для одного байта.
//
// Отражённый полином 0xEDB88320 (это зеркало 0x04C11DB7). Начальное значение
// 0xFFFFFFFF и финальная инверсия результата делаются СНАРУЖИ, в MAC-е; здесь
// обрабатывается ровно один байт: байт вносится в младшие 8 бит, затем 8 сдвигов.
//
// Якорь корректности: если пропустить строку "123456789" (байты 0x31..0x39)
// через цепочку из 8 шагов при init = 0xFFFFFFFF и финальном XOR 0xFFFFFFFF,
// получится 0xCBF43926 — стандартная контрольная величина CRC-32.
`timescale 1ns/1ps

module crc32 (
    input  wire [31:0] crc_in,
    input  wire [7:0]  data,
    output wire [31:0] crc_out
);
    function [31:0] step;
        input [31:0] c;
        integer i;
        begin
            step = c;
            for (i = 0; i < 8; i = i + 1)
                step = step[0] ? ((step >> 1) ^ 32'hEDB8_8320) : (step >> 1);
        end
    endfunction

    assign crc_out = step(crc_in ^ {24'b0, data});
endmodule

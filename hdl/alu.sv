`timescale 1ns/1ps

`include "6502_defs.vh"

module alu (
    input  logic [2:0] op,
    input  logic [7:0] ai, bi,
    input  logic ci,
    output logic [7:0] out,
    output logic aluV,aluC
    );

    logic [7:0] aorb, aandb;

    always @(*) begin
        aorb = ai | bi;
        aandb = ai & bi;
        aluC = 0;
        
        case(op)
            ALU_AND:    out = aandb;
            ALU_OR:     out = aorb;
            ALU_XOR:    out = ai ^ bi;
            ALU_SR:     {out, aluC} = {ci, aorb};
            ALU_SL:     {aluC, out} = {aorb, ci};
            // verilator lint_off WIDTH
            ALU_ADD:    {aluC, out} = ai + bi + ci;
            // verilator lint_on WIDTH
            default:    out = '0;
        endcase

        //https://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
        aluV = (ai[7] ^ out[7]) && (bi[7] ^ out[7]);
    end

endmodule

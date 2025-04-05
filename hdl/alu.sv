`timescale 1ns/1ps

`include "6502_defs.vh"

module alu (
    input  logic clk, rst,
    input  logic [8:0] op,
    input  logic [7:0] ai, bi,
    input  logic ci,
    output logic [7:0] out,
    output logic sumC, sumV, bpage
    );

    logic [7:0] a,b;
    logic c;
    wire [3:0] flags = op[8:5];             // flags: {invert b, zero b, carry 1, carry p}
    wire [7:0] bz = flags[2] ? 8'b0 : bi;   // bi possibly zeroed
    wire [7:0] bzi = flags[3] ? ~bz : bz;   // bi possibly inverted and/or zeroed

    logic [4:0] alu_op=0;                     // ops: {sum, shiftr,  xor, or, and}

    always @(posedge clk ) begin
        // $display("alu: %x", op[4:0]);
        if (rst) begin
            alu_op <= ALU_NOP;
            a <= 0;
            b <= 0;
            c <= 0;
        end else begin 
            alu_op <= op[4:0];
            a <= ai;
            b <= bzi;
            c <= (flags[0] & ci) | flags[1];
        end
    end

    logic [7:0] sum;
    // verilator lint_off WIDTH
    assign {sumC, sum} = a + b + c;
    // verilator lint_on WIDTH

    //https://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
    assign sumV = (a[7] ^ sum[7]) && (b[7] ^ sum[7]);

    // branch crosses page (a+b>0xff or a+b<0x00?)
    assign bpage = (b[7] == sum[7]) && (b[7] ^ a[7]);

    always_comb begin
        unique case(1'b1)
            alu_op[0]:  out = a & b;
            alu_op[1]:  out = a | b;
            alu_op[2]:  out = a ^ b;
            alu_op[3]:  out = {c, a[7:1]};
            alu_op[4]:  out = sum;
            default:    out = a;
        endcase 
    end

endmodule

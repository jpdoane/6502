`timescale 1ns/1ps

`include "6502_defs.vh"

module alu (
    input  logic [8:0] op,
    input  logic [7:0] ai, bi,
    input  logic ci,
    output logic [7:0] out,
    output logic sumC, sumV, sumB
    );

    wire [3:0] flags = op[8:5];             // flags: {invert b, zero b, carry 1, carry p}
    wire [7:0] bz = flags[2] ? 8'b0 : bi;   // bi possibly zeroed
    wire [7:0] bzi = flags[3] ? ~bz : bz;   // bi possibly inverted and/or zeroed

    logic [7:0] a,b;
    logic c;
    assign a = ai;
    assign b = bzi;
    assign c = (flags[0] & ci) | flags[1];

    logic [8:0] sum_result;
    logic [7:0] sum;
    assign sum_result = {1'b0,a} + {1'b0,b} + {8'b0,c};
    assign sum = sum_result[7:0];
    assign sumC = sum_result[8];

    //https://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
    assign sumV = (a[7] ^ sum[7]) && (b[7] ^ sum[7]);

    // branch crosses page (a+b>0xff or a+b<0x00?)
    assign sumB = (b[7] == sum[7]) && (b[7] ^ a[7]);

    always_comb begin
        unique case(1'b1)
            op[0]:    out = a & b;
            op[1]:    out = a | b;
            op[2]:    out = a ^ b;
            op[3]:    out = {c, a[7:1]};
            op[4]:    out = sum;
            default:  out = a;
        endcase 
    end

endmodule

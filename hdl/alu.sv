`timescale 1ns/1ps

`include "6502_defs.vh"

module alu (
    input  logic clk, rst,
    input  logic [4:0] op,
    input  logic [7:0] ai, bi,
    input  logic ci,
    output logic [7:0] out,
    output logic sumC, sumV, bpage
    );

    logic [4:0] op_r;
    logic [7:0] a_r,b_r;
    logic c_r;
    always @(posedge clk ) begin
        if (rst) begin
            op_r <= ALU_NOP;
            a_r <= 0;
            b_r <= 0;
            c_r <= 0;
        end else begin 
            op_r <= op;
            a_r <= ai;
            b_r <= bi;
            c_r <= op[4] & ci;
        end
    end

    // adder logic
    logic [7:0] a,b, sum;
    logic c;
    always_comb begin
        a = a_r;
        b = b_r;
        c = c_r;
        
        if (op_r[1]) begin // inc/dec commands
            b = 0;
            c = !op_r[0];     // set carry on inc, not on dec
        end
            else if(op_r[2]) c = 1; // op[2] sets carry in to 1

        if (op_r[0]) b = ~b;   // invert port b for sub/dec

        // verilator lint_off WIDTH
        {sumC, sum} = a + b + c;
        // verilator lint_on WIDTH

        //https://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
        sumV = (a[7] ^ sum[7]) && (b[7] ^ sum[7]);

        // branch crosses page (a+b>0xff or a+b<0x00?)
        bpage = (b[7] == sum[7]) && (b[7] ^ a[7]);
    end

    // other logic
    logic [7:0] out_logical;
    always_comb begin
        case(op_r[2:0])
            ALU_AND[2:0]:   out_logical = a_r & b_r;
            ALU_ORA[2:0]:   out_logical = a_r | b_r;
            ALU_XOR[2:0]:   out_logical = a_r ^ b_r;
            ALU_LSR[2:0]:   out_logical = {c_r, a_r[7:1]};
            default:        out_logical = a_r;
        endcase    
    end

    assign out = op_r[3] ? sum : out_logical;

endmodule

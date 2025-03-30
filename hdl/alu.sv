`timescale 1ns/1ps

`include "6502_defs.vh"

module alu (
    input  logic clk, rst,
    input  logic [5:0] op,
    input  logic [7:0] ai, bi,
    input  logic ci,
    output logic [7:0] out,
    output logic [7:0] status,
    output logic bpage
    );

    logic [7:0] a,b, result;
    logic c, aluC, aluV, bit_op;
                
    always @(*) begin
        aluC = 0;
        bit_op = 0;

        // op[5] activates port b for unary ops (inc/dec/sr/sl)
        a = op[5] ? bi : ai;
        b = bi;
        c = op[4] & ci;

        if (op[3]) begin // sum ops        

            if (op[1]) begin // inc/dec commands
                b = 0;
                c = !op[0];     // set carry on inc, not on dec
            end
                else if(op[2]) c = 1; // op[2] sets carry in to 1

            if (op[0]) b = ~b;   // invert port b for sub/dec

            // verilator lint_off WIDTH
            {aluC, result} = a + b + c;
            // verilator lint_on WIDTH

            //https://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
            aluV = (a[7] ^ result[7]) && (b[7] ^ result[7]);

        end else begin

            case(op[2:0])
                ALU_AND[2:0]:   result = ai & bi;
                ALU_ORA[2:0]:   result = ai | bi;
                ALU_XOR[2:0]:   result = ai ^ bi;
                ALU_LSR[2:0]:   {result, aluC} = {c, a};
                ALU_ASL[2:0]:   {aluC, result} = {a, c};
                ALU_BIT[2:0]:   begin
                                    result = ai & bi;
                                    bit_op = 1;
                                end
                default:        result = ai;
            endcase    

            aluV = b[6];        // only used by BIT
        end

    end

    // special logic used used for branch page fault detection
    logic bpage_up, bpage_down;
    assign bpage_up = a[7] & !b[7] & !result[7]; // crossed to next page if base>127, offset>0, and result <= 127
    assign bpage_down = !a[7] & b[7] & result[7]; // crossed to prev page if base<=127, offset<0, and result > 127

    always @(posedge clk ) begin
        if(rst) begin
            status <= 0;
            out <= 0;
            bpage <= 0;
        end else begin
            status[7] <= bit_op ? bi[7] : result[7];    // N
            status[6] <= aluV;                          // V
            status[1] <= result == 0;                   // Z
            status[0] <= aluC;                          // C
            out <= result;
            bpage <= bpage_up | bpage_down;
        end
    end

endmodule

`timescale 1ns/1ps

`include "6502_defs.vh"

module alu (
    input  logic [5:0] op,
    input  logic [7:0] ai, bi,
    input  logic ci,
    output logic [7:0] out,
    output logic aluV,aluC
    );

    logic [7:0] a,b;
    logic c;

    always @(*) begin
        aluC = 0;
        aluV = 0;

        // op[5] activates port b for unary ops (inc/dec/sr/sl)
        a = op[5] ? bi : ai;
        b = bi;
        c = op[4] & ci;

        if (op[3]) begin // sum ops        

            if (op[1]) begin // inc/dec commands
                b = 0;
                c = !op[0];     // set carry on inc, not on dec
            end else begin
                c = c | op[2];  // op[2] sets carry in to 1
            end
            if (op[0]) begin   // invert port b for sub/dec
                b = ~b;
            end

            // verilator lint_off WIDTH
            {aluC, out} = a + b + c;
            // verilator lint_on WIDTH

            //https://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
            aluV = (a[7] ^ out[7]) && (b[7] ^ out[7]);

        end else begin
            case(op[2:0])
                ALU_AND[2:0]: out = ai & bi;
                ALU_ORA[2:0]:  out = ai | bi;
                ALU_XOR[2:0]: out = ai ^ bi;
                ALU_LSR[2:0]: {out, aluC} = {c, a};
                ALU_ASL[2:0]: {aluC, out} = {a, c};
                default:    out = a;
            endcase    
        end
    end

endmodule

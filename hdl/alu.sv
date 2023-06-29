module alu (
    input  logic [7:0] ai, bi,
    input  logic ci,
    input  logic [2:0] op,

    output logic [7:0] out,
    output logic N, V, Z, C
    );

    logic [7:0] aorb;

    always @(*) begin
        aorb = ai | bi;
        out = 0;
        C = 0;

        case(op)
            ALU_BIT,
            ALU_AND:    out = ai & bi;
            ALU_OR:     out = aorb;
            ALU_XOR:    out = ai ^ bi;
            // unary shifts operate on ai|bi, so unused port must be zeroed
            ALU_SR:     {out, C} = {ci, aorb};
            ALU_SL:     {C, out} = {aorb, ci};
            // verilator lint_off WIDTH
            ALU_ADD:    {C, out} = ai + bi + ci;
            // verilator lint_on WIDTH
            default:    out = '0;
        endcase

        Z = ~|out;

        if(op == ALU_BIT) begin
            N = bi[7];
            V = bi[6];
        end else begin
            N = out[7];
            //https://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
            V = (ai[7] ^ out[7]) && (bi[7] ^ out[7]);
        end   
    end

endmodule

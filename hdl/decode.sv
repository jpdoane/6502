`timescale 1ns/1ps
`include "6502_defs.vh"

module decode (
    input  logic [7:0] op,
    output logic [4:0] op_type,
    output logic [6:0] src, dst,
    output logic [4:0] alu_op,
    output logic alu_en,                            // alu ctl
    output logic single_byte,                       // single byte op
    output logic idx_XY,                            // index on X vs Y
    output logic stack_ap,
    output logic and_op,
    output logic bit_op,
    output logic sl_op, sr_op,
    output logic clc, cli, clv, cld, sec, sei, sed,
    output logic [7:0] result_mask                  // set/clear flags
    );

    logic [18:0] ctl_flags;
    assign {dst, src, alu_op} = ctl_flags;

    // special case flags
    logic sum_op, cmp_op, rot_op, inc_op, take_branch;
    
    logic upV, upZ, upC;

    // decode datapath and alu op
     /* verilator lint_off CASEOVERLAP */
    always_comb begin
        unique casez(op)     //ctl_flags = {dst, src,  alu_op}
            8'b0??_010_00:  ctl_flags = {REG_Z, REG_ADD, ALU_NOP};     // PUS,PUL
            8'b101_010_00:  ctl_flags = {REG_Y, REG_A, ALU_NOP};       // TAY
            8'b111_010_00:  ctl_flags = {REG_X, REG_X, ALU_INC};       // INX
            8'b110_010_00:  ctl_flags = {REG_Y, REG_Y, ALU_INC};       // INY
            8'b100_010_00:  ctl_flags = {REG_Y, REG_Y, ALU_DEC};       // DEY
            8'b001_0?1_00:  ctl_flags = {REG_Z, REG_A, ALU_AND};       // BIT
            8'b100_110_00:  ctl_flags = {REG_A, REG_Y, ALU_NOP};       // TYA
            8'b100_010_10:  ctl_flags = {REG_A, REG_X, ALU_NOP};       // TXA
            8'b100_110_10:  ctl_flags = {REG_S, REG_X, ALU_NOP};       // TXS
            8'b101_010_10:  ctl_flags = {REG_X, REG_A, ALU_NOP};       // TAX
            8'b101_110_10:  ctl_flags = {REG_X, REG_S, ALU_NOP};       // TSX
            8'b100_??1_00:  ctl_flags = {REG_D, REG_Y, ALU_NOP};       // STY
            8'b100_??1_10:  ctl_flags = {REG_D, REG_X, ALU_NOP};       // STX
            8'b101_000_00,
            8'b101_??1_00:  ctl_flags = {REG_Y, REG_D, ALU_NOP};       // LDY
            8'b101_??1_10,
            8'b101_000_10:  ctl_flags = {REG_X, REG_D, ALU_NOP};       // LDX
            8'b100_???_?1:  ctl_flags = {REG_D, REG_A, ALU_NOP};       // STA
            8'b101_???_?1:  ctl_flags = {REG_A, REG_D, ALU_NOP};       // LDA
            8'b11?_011_00,
            8'b11?_00?_00:  ctl_flags = {REG_Z, op[5]? REG_X : REG_Y,
                                                       ALU_CMP};       // CPX, CPY
            8'b110_???_?1:  ctl_flags = {REG_Z, REG_A, ALU_CMP};       // CMP
            8'b000_???_?1:  ctl_flags = {REG_A, REG_A, ALU_ORA};       // ORA
            8'b001_???_?1:  ctl_flags = {REG_A, REG_A, ALU_AND};       // AND
            8'b010_???_?1:  ctl_flags = {REG_A, REG_A, ALU_XOR};       // EOR
            8'b011_???_?1:  ctl_flags = {REG_A, REG_A, ALU_ADC};       // ADC
            8'b111_???_?1:  ctl_flags = {REG_A, REG_A, ALU_SBC};       // SBC
            8'b000_010_10:  ctl_flags = {REG_A, REG_A, ALU_SUM};       // ASL, A  (implemented as M+M) 
            8'b001_010_10:  ctl_flags = {REG_A, REG_A, ALU_ADC};       // ROL, A  (implemented as M+M+C)
            8'b010_010_10:  ctl_flags = {REG_A, REG_A, ALU_LSR};       // LSR, A  
            8'b011_010_10:  ctl_flags = {REG_A, REG_A, ALU_ROR};       // ROR, A  
            8'b110_???_10:  ctl_flags = {op[2]? REG_D : REG_X,
                                                op[2]? REG_D : REG_X,
                                                       ALU_DEC};       // DEX, DEC rmw
            8'b111_??1_10:  ctl_flags = {REG_D, REG_D, ALU_INC};       // INC rmw
            8'b000_??1_10:  ctl_flags = {REG_D, REG_D, ALU_SUM};       // ASL rmw (implemented as M+M)
            8'b001_??1_10:  ctl_flags = {REG_D, REG_D, ALU_ADC};       // ROL rmw (implemented as M+M+C)
            8'b010_??1_10:  ctl_flags = {REG_D, REG_D, ALU_LSR};       // LSR rmw
            8'b011_??1_10:  ctl_flags = {REG_D, REG_D, ALU_ROR};       // ROR rmw
            default:        ctl_flags = {REG_Z, REG_Z, ALU_NOP};       // ctrl flow, set/clear, NOP
        endcase

        alu_en = (alu_op != ALU_NOP);
        sum_op = &alu_op[4:3];
        cmp_op = (alu_op == ALU_CMP);
        sl_op = op ==? 8'b00?_???_10;
        sr_op = op ==? 8'b01?_???_10;
        and_op = op ==? 8'b001_???_?1;
        bit_op = op ==? 8'b001_0?1_00;

        stack_ap = op[6]; // high for PHA,PLA, low for PHP,PLP

        // update status flags (BIT opcodes are special case handled elsewhere...)
        // update N&Z bits on any write to a,x,y regs and all alu ops
        unique case(dst)
            REG_A:      upZ=1;
            REG_X:      upZ=1;
            REG_Y:      upZ=1;
            default:    upZ=alu_en;
        endcase

        // set v flag on ADC, SBC and BIT
        upV = sum_op & ~sl_op;
        upC = sum_op | cmp_op | sl_op;
        // upC = sum_op | cmp_op | shift_op | rot_op;
        result_mask = {upZ & ~bit_op, upV, 4'b0, upZ, upC};

        // set and clear masks
        clc = op == 8'b000_110_00;
        cli = op == 8'b010_110_00;
        clv = op == 8'b101_110_00;
        cld = op == 8'b110_110_00;
        sec = op == 8'b001_110_00;
        sei = op == 8'b011_110_00;
        sed = op == 8'b111_110_00;

        // single byte opcodes: b = 2 or 6 && c = 0 or 2
        single_byte = (op == 8'h0) || (op ==? 8'b???_?10_?0);

        // X vs Y indexing
        idx_XY = (op ==? 8'b???_1?0_?1 || op ==? 8'b10?_1?1_1?) ? 1'b0 : 1'b1;

        // decode control flow and memory access pattern types
        // https://www.masswerk.at/6502/6502_instruction_set.html#layout
        casez(op)
            8'b000_000_00:  op_type = OP_BRK;          
            8'b001_000_00:  op_type = OP_JSR;          
            8'b010_000_00:  op_type = OP_RTI;          
            8'b011_000_00:  op_type = OP_RTS;          
            8'b???_000_?1:  op_type = OP_XIN;         
            8'b1??_000_?0,
            8'b???_010_?1:  op_type = OP_IMM;          
            8'b???_001_??:  op_type = OP_ZPG;          
            8'b0?0_010_00:  op_type = OP_PUS;         
            8'b0?1_010_00:  op_type = OP_PUL;          
            8'b???_?10_?0:  op_type = OP_IMP;         
            8'b010_011_00:  op_type = OP_JUM;         
            8'b011_011_00:  op_type = OP_JIN;
            8'b???_011_??:  op_type = OP_ABS;          
            8'b???_100_00:  op_type = OP_BRA;           
            8'b???_100_?1:  op_type = OP_INY;         
            8'b???_101_??:  op_type = OP_ZXY;        
            8'b???_110_?1,
            8'b???_111_??:  op_type = OP_AXY;        
            default:        op_type = OP_JAM;
        endcase

    end
    /* verilator lint_on CASEOVERLAP */


endmodule

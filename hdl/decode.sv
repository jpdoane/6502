`timescale 1ns/1ps
`include "6502_defs.vh"

module decode (
    input  logic [7:0] opcode,
    input  logic [7:0] pstatus,
    output logic [4:0] op_type,
    output logic [3:0] dst, a_src, b_src,
    output logic [2:0] alu_op,
    output logic alu_en, alu_cin, upNZ, upV, upC, bit_op,   // alu ctl
    output logic mem_rd, mem_wr,                    // mem access patterns
    output logic take_branch,
    output logic single_byte,                       // single byte opcode
    output logic idx_XY,                            // index on X vs Y
    output logic [7:0] set_mask, clear_mask         // set/clear flags
    );

    logic [14:0] ctl_flags;
    assign {dst, a_src, b_src, alu_op} = ctl_flags;

    // special case flags
    logic adc_sbc_op, cmp_op, rot_op, shift_op, inc_op;
    
    // decode datapath and alu opcode
     /* verilator lint_off CASEOVERLAP */
    always_comb begin
        casez(opcode)     //ctl_flags = {dst,   a_src, b_src, alu_op}
            8'b0??_000_00:  ctl_flags = {REG_Z, REG_Z, REG_Z, ALU_NOP};       // control flow and special ops
            8'b000_010_00:  ctl_flags = {REG_D, REG_Z, REG_P, ALU_NOP};       // PHP 
            8'b010_010_00:  ctl_flags = {REG_D, REG_A, REG_Z, ALU_NOP};       // PHA
            8'b001_010_00:  ctl_flags = {REG_P, REG_Z, REG_D, ALU_NOP};       // PLP
            8'b011_010_00:  ctl_flags = {REG_A, REG_Z, REG_D, ALU_NOP};       // PLA
            8'b101_010_00:  ctl_flags = {REG_Y, REG_A, REG_Z, ALU_NOP};       // TAY
            8'b111_010_00:  ctl_flags = {REG_X, REG_X, REG_Z, ALU_ADD};       // INX
            8'b110_010_00:  ctl_flags = {REG_Y, REG_Y, REG_Z, ALU_ADD};       // INY
            8'b110_010_10:  ctl_flags = {REG_X, REG_X, RE_NZ, ALU_ADD};       // DEX
            8'b100_010_00:  ctl_flags = {REG_Y, REG_Y, RE_NZ, ALU_ADD};       // DEY
            8'b001_0?1_00:  ctl_flags = {REG_Z, REG_A, REG_D, ALU_AND};       // BIT
            8'b100_110_00:  ctl_flags = {REG_A, REG_Y, REG_Z, ALU_NOP};       // TYA
            8'b100_010_10:  ctl_flags = {REG_A, REG_X, REG_Z, ALU_NOP};       // TXA
            8'b100_110_10:  ctl_flags = {REG_S, REG_X, REG_Z, ALU_NOP};       // TXS
            8'b101_010_10:  ctl_flags = {REG_X, REG_A, REG_Z, ALU_NOP};       // TAX
            8'b101_110_10:  ctl_flags = {REG_X, REG_S, REG_Z, ALU_NOP};       // TSX
            8'b???_1?0_00:  ctl_flags = {REG_Z, REG_Z, REG_Z, ALU_NOP};       // branch & mask ops
            8'b100_??1_00:  ctl_flags = {REG_D, REG_Y, REG_Z, ALU_NOP};       // STY
            8'b100_??1_10:  ctl_flags = {REG_D, REG_X, REG_Z, ALU_NOP};       // STX
            8'b101_???_00:  ctl_flags = {REG_Y, REG_Z, REG_D, ALU_NOP};       // LDY
            8'b100_???_?1:  ctl_flags = {REG_D, REG_A, REG_Z, ALU_NOP};       // STA
            8'b101_???_?1:  ctl_flags = {REG_A, REG_Z, REG_D, ALU_NOP};       // LDA
            8'b101_???_10:  ctl_flags = {REG_X, REG_Z, REG_D, ALU_NOP};       // LDX
            8'b110_0??_00:  ctl_flags = {REG_Z, REG_Y, RE_ND, ALU_ADD};       // CPY
            8'b111_0??_00:  ctl_flags = {REG_Z, REG_X, RE_ND, ALU_ADD};       // CPX
            8'b110_???_?1:  ctl_flags = {REG_Z, REG_A, RE_ND, ALU_ADD};       // CMP
            8'b111_010_10:  ctl_flags = {REG_Z, REG_Z, REG_Z, ALU_NOP};       // NOP
            8'b000_???_?1:  ctl_flags = {REG_A, REG_A, REG_D, ALU_OR };       // ORA
            8'b001_???_?1:  ctl_flags = {REG_A, REG_A, REG_D, ALU_AND};       // AND
            8'b010_???_?1:  ctl_flags = {REG_A, REG_A, REG_D, ALU_XOR};       // EOR
            8'b011_???_?1:  ctl_flags = {REG_A, REG_A, REG_D, ALU_ADD};       // ADC
            8'b111_???_?1:  ctl_flags = {REG_A, REG_A, RE_ND, ALU_ADD};       // SBC
            8'b000_010_10:  ctl_flags = {REG_A, REG_A, REG_Z, ALU_SL };       // ASL, A  
            8'b001_010_10:  ctl_flags = {REG_A, REG_A, REG_Z, ALU_SL };       // ROL, A  
            8'b010_010_10:  ctl_flags = {REG_A, REG_A, REG_Z, ALU_SR };       // LSR, A  
            8'b011_010_10:  ctl_flags = {REG_A, REG_A, REG_Z, ALU_SR };       // ROR, A  
            8'b110_???_10:  ctl_flags = {REG_D, RE_NZ, REG_D, ALU_ADD};       // DEC rmw
            8'b111_???_10:  ctl_flags = {REG_D, REG_Z, REG_D, ALU_ADD};       // INC rmw
            8'b000_???_10:  ctl_flags = {REG_D, REG_Z, REG_D, ALU_SL };       // ASL rmw
            8'b001_???_10:  ctl_flags = {REG_D, REG_Z, REG_D, ALU_SL };       // ROL rmw
            8'b010_???_10:  ctl_flags = {REG_D, REG_Z, REG_D, ALU_SR };       // LSR rmw
            8'b011_???_10:  ctl_flags = {REG_D, REG_Z, REG_D, ALU_SR };       // ROR rmw
            default:        ctl_flags = {REG_Z, REG_Z, REG_Z, ALU_NOP};       // nop
        endcase

        mem_rd = (b_src == REG_D) || (b_src == RE_ND);
        mem_wr = dst == REG_D;

        alu_en = (alu_op != ALU_NOP);
        adc_sbc_op = opcode ==? 8'b?11_???_?1;
        cmp_op = opcode ==? 8'b11?_0??_00 || opcode ==? 8'b110_???_?1;
        rot_op = opcode ==? 8'b0?1_???_10;
        shift_op = opcode ==? 8'b0?0_???_10;
        bit_op = opcode ==? 8'b001_0?1_00;
        inc_op = opcode ==? 8'b11?_010_00 || opcode ==? 8'b111_???_10;

        // update status flags (BIT opcodes are special case handled elsewhere...)
        // update N&Z bits on any write to a,x,y regs and all alu ops
        case(dst)
            REG_A:      upNZ=1;
            REG_X:      upNZ=1;
            REG_Y:      upNZ=1;
            default:    upNZ=alu_en;
        endcase
        // set v flag on ADC and SBC
        upV = adc_sbc_op;
        // set c flag on ADC,SBC, and rotate/shift ops
        upC = adc_sbc_op | cmp_op | shift_op | rot_op;

        // set carry bit on cmp & inc ops, and set to p[0] on ADC,SBC, and rotate ops
        alu_cin = inc_op | cmp_op | ((adc_sbc_op | rot_op) & pstatus[0]);

        // set and clear masks
        clear_mask = 0;
        set_mask = 0;
        casez(opcode)
            8'b000_110_00: clear_mask = FL_C; // CLC
            8'b010_110_00: clear_mask = FL_I; // CLI
            8'b101_110_00: clear_mask = FL_V; // CLV
            8'b110_110_00: clear_mask = FL_D; // CLD
            8'b001_110_00: set_mask = FL_C;   // SEC
            8'b011_110_00: set_mask = FL_I;   // SEI
            8'b111_110_00: set_mask = FL_D;   // SED
            default:    begin end
        endcase

        // single byte opcodes: b = 2 or 6 && c = 0 or 2
        single_byte = (opcode == 8'h0) || (opcode ==? 8'b???_?10_?0);

        // X vs Y indexing
        idx_XY = (opcode ==? 8'b???_1?0_?1 || opcode ==? 8'b10?_1?1_1?) ? IDX_Y : IDX_X;

        // branch logic
        case(opcode[7:6])
            2'h0:   take_branch = pstatus[7] ^ !opcode[5]; // BPL, BMI
            2'h1:   take_branch = pstatus[6] ^ !opcode[5]; // BVC, BVS
            2'h2:   take_branch = pstatus[0] ^ !opcode[5]; // BCC, BCS
            2'h3:   take_branch = pstatus[1] ^ !opcode[5]; // BNE, BEQ
        endcase

        // decode control flow and memory access pattern types
        // https://www.masswerk.at/6502/6502_instruction_set.html#layout
        casez(opcode)
            8'b000_000_00:  op_type = OP_BRK;          
            8'b001_000_00:  op_type = OP_JSR;          
            8'b010_000_00:  op_type = OP_RTI;          
            8'b011_000_00:  op_type = OP_RTS;          
            8'b???_000_?1:  op_type = OP_XIN;         
            8'b1??_000_?0,
            8'b???_010_?1:  op_type = OP_IMM;          
            8'b???_001_??:  op_type = OP_ZPG;          
            8'b0?0_010_00:  op_type = OP_PUS;         
            8'b0?1_010_00:  op_type = OP_POP;          
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

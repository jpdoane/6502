`timescale 1ns/1ps
`include "6502_defs.vh"

module decode (
    input  logic [7:0] opcode,
    input  logic [7:0] pstatus,
    output logic [4:0] op_type,
    output logic [5:0] src, dst,
    output logic [5:0] alu_op,
    output logic alu_en, upNZ, upV, upC, bit_op,   // alu ctl
    output logic single_byte,                       // single byte opcode
    output logic idx_XY,                            // index on X vs Y
    output logic stack, stack_ap,
    output logic [7:0] set_mask, clear_mask         // set/clear flags
    );

    logic [17:0] ctl_flags;
    assign {dst, src, alu_op} = ctl_flags;

    // special case flags
    logic adc_sbc_op, cmp_op, rot_op, shift_op, inc_op, take_branch;

    // decode datapath and alu opcode
     /* verilator lint_off CASEOVERLAP */
    always_comb begin
        casez(opcode)     //ctl_flags = {dst, src,  alu_op}
            8'b0??_000_00:  ctl_flags = {REG_Z, REG_Z, ALU_NOP};       // control flow and special ops
            8'b0??_010_00:  ctl_flags = {REG_Z, REG_Z, ALU_NOP};       // PUS
            8'b101_010_00:  ctl_flags = {REG_Y, REG_A, ALU_NOP};       // TAY
            8'b111_010_00:  ctl_flags = {REG_X, REG_X, ALU_INC};       // INX
            8'b110_010_00:  ctl_flags = {REG_Y, REG_Y, ALU_INC};       // INY
            8'b110_010_10:  ctl_flags = {REG_X, REG_X, ALU_DEC};       // DEX
            8'b100_010_00:  ctl_flags = {REG_Y, REG_Y, ALU_DEC};       // DEY
            8'b001_0?1_00:  ctl_flags = {REG_Z, REG_A, ALU_AND};       // BIT
            8'b100_110_00:  ctl_flags = {REG_A, REG_Y, ALU_NOP};       // TYA
            8'b100_010_10:  ctl_flags = {REG_A, REG_X, ALU_NOP};       // TXA
            8'b100_110_10:  ctl_flags = {REG_S, REG_X, ALU_NOP};       // TXS
            8'b101_010_10:  ctl_flags = {REG_X, REG_A, ALU_NOP};       // TAX
            8'b101_110_10:  ctl_flags = {REG_X, REG_S, ALU_NOP};       // TSX
            8'b???_1?0_00:  ctl_flags = {REG_Z, REG_Z, ALU_NOP};       // branch & mask ops
            8'b100_??1_00:  ctl_flags = {REG_D, REG_Y, ALU_NOP};       // STY
            8'b100_??1_10:  ctl_flags = {REG_D, REG_X, ALU_NOP};       // STX
            8'b101_???_00:  ctl_flags = {REG_Y, REG_D, ALU_NOP};       // LDY
            8'b100_???_?1:  ctl_flags = {REG_D, REG_A,  ALU_NOP};       // STA
            8'b101_???_?1:  ctl_flags = {REG_A, REG_D, ALU_NOP};       // LDA
            8'b101_???_10:  ctl_flags = {REG_X, REG_D, ALU_NOP};       // LDX
            8'b110_0??_00:  ctl_flags = {REG_Z, REG_Y, ALU_CMP};       // CPY
            8'b111_0??_00:  ctl_flags = {REG_Z, REG_X, ALU_CMP};       // CPX
            8'b110_???_?1:  ctl_flags = {REG_Z, REG_A, ALU_CMP};       // CMP
            8'b111_010_10:  ctl_flags = {REG_Z, REG_Z, ALU_NOP};       // NOP
            8'b000_???_?1:  ctl_flags = {REG_A, REG_A, ALU_ORA};       // ORA
            8'b001_???_?1:  ctl_flags = {REG_A, REG_A, ALU_AND};       // AND
            8'b010_???_?1:  ctl_flags = {REG_A, REG_A, ALU_XOR};       // EOR
            8'b011_???_?1:  ctl_flags = {REG_A, REG_A, ALU_ADC};       // ADC
            8'b111_???_?1:  ctl_flags = {REG_A, REG_A, ALU_SBC};       // SBC
            8'b000_010_10:  ctl_flags = {REG_A, REG_A, ALU_ASL};       // ASL, A  
            8'b001_010_10:  ctl_flags = {REG_A, REG_A, ALU_ROL};       // ROL, A  
            8'b010_010_10:  ctl_flags = {REG_A, REG_A, ALU_LSR};       // LSR, A  
            8'b011_010_10:  ctl_flags = {REG_A, REG_A, ALU_ROR};       // ROR, A  
            8'b110_???_10:  ctl_flags = {REG_D, REG_D, ALU_DEC | ALU_OPB };       // DEC rmw
            8'b111_???_10:  ctl_flags = {REG_D, REG_D, ALU_INC | ALU_OPB };       // INC rmw
            8'b000_???_10:  ctl_flags = {REG_D, REG_D, ALU_ASL | ALU_OPB };       // ASL rmw
            8'b001_???_10:  ctl_flags = {REG_D, REG_D, ALU_ROL | ALU_OPB };       // ROL rmw
            8'b010_???_10:  ctl_flags = {REG_D, REG_D, ALU_LSR | ALU_OPB };       // LSR rmw
            8'b011_???_10:  ctl_flags = {REG_D, REG_D, ALU_ROR | ALU_OPB };       // ROR rmw
            default:        ctl_flags = {REG_Z, REG_Z, ALU_NOP};       // nop
        endcase

        alu_en = (alu_op != ALU_NOP);
        adc_sbc_op = &alu_op[4:3];
        cmp_op = opcode ==? 8'b11?_0??_00 || opcode ==? 8'b110_???_?1;
        rot_op = opcode ==? 8'b0?1_???_10;
        shift_op = opcode ==? 8'b0?0_???_10;
        bit_op = opcode ==? 8'b001_0?1_00;
        // inc_op = opcode ==? 8'b11?_010_00 || opcode ==? 8'b111_???_10;

        stack = opcode ==? 8'b0??_010_00;
        stack_ap = opcode[6]; // high for PHA,PLA, low for PHP,PLP

        // update status flags (BIT opcodes are special case handled elsewhere...)
        // update N&Z bits on any write to a,x,y regs and all alu ops
        case(dst)
            REG_A:      upNZ=1;
            REG_X:      upNZ=1;
            REG_Y:      upNZ=1;
            default:    upNZ=alu_en & !stack;
        endcase
        // set v flag on ADC and SBC
        upV = adc_sbc_op;
        // set c flag on ADC,SBC, and rotate/shift ops
        upC = adc_sbc_op | cmp_op | shift_op | rot_op;

        // // set carry bit on cmp & inc ops, and set to p[0] on ADC,SBC, and rotate ops
        // alu_cin = inc_op | cmp_op | ((adc_sbc_op | rot_op) & pstatus[0]);

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
        idx_XY = (opcode ==? 8'b???_1?0_?1 || opcode ==? 8'b10?_1?1_1?) ? 1'b0 : 1'b1;

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
            8'b0?1_010_00:  op_type = OP_PUL;          
            8'b???_?10_?0:  op_type = OP_IMP;         
            8'b010_011_00:  op_type = OP_JUM;         
            8'b011_011_00:  op_type = OP_JIN;
            8'b???_011_??:  op_type = OP_ABS;          
            8'b???_100_00:  op_type = take_branch ? OP_BRA : OP_BNT;           
            8'b???_100_?1:  op_type = OP_INY;         
            8'b???_101_??:  op_type = OP_ZXY;        
            8'b???_110_?1,
            8'b???_111_??:  op_type = OP_AXY;        
            default:        op_type = OP_JAM;
        endcase



    end
    /* verilator lint_on CASEOVERLAP */


endmodule

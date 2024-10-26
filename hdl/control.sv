`timescale 1ns/1ps
`include "6502_defs.vh"

module control #()
    (
    input  logic clk_m1,
    input  logic clk_m2,
    input  logic rst,
    input  logic [7:0] data_i,
    );

    localparam REGX = 2'h0;
    localparam REGY = 2'h1;
    localparam REGS = 2'h2;
    localparam REGA = 2'h3;

    // data_reg: input data registered on m2

    reg rstg, intg;


    // branch logic
    logic take_branch;
    always_comb begin
        case(opcode[7:5]) //op_a
            3'h0:   take_branch = !p[7]; // BPL
            3'h1:   take_branch = p[7];  // BMI
            3'h2:   take_branch = !p[6]; // BVC
            3'h3:   take_branch = p[6];  // BVS
            3'h4:   take_branch = !p[0]; // BCC
            3'h5:   take_branch = p[0];  // BCS
            3'h6:   take_branch = !p[1]; // BNE
            3'h7:   take_branch = p[1];  // BEQ
        endcase
    end

    // predecode logic
    // valid only on instruction fetch (sync=1)
    logic [7:0] ir_in;
    logic two_cycle;
    always_comb begin
        ir_in = (rstg || intg) ? 0 : data_in;

        // early decode 2 cycle instructions to set T0+2 special case for timing state
        // TZPRE in block diagram
        two_cycle = (ir_in == 8'b1??_000_?0) ||                 // LD/CP IMM
                    (ir_in[4:0] == 5'b010??) ||                 // IMM or IMPL
                    (ir_in[4:0] == 5'b110?0) ||                 // IMPL
                   ((ir_in[4:0] == 5'b10000) && !take_branch);  // branch not taken
    end

    // ir
    logic sync;
    always @(posedge clk_m1 ) begin
        if (i_rst) ir <= 0;
        else if (rdy && sync)
            ir <= ir_in;
    end

    // 2-byte, 2 cycle opcode summary:
    // https://www.nesdev.org/wiki/Visual6502wiki/6502_Timing_States
    //
    //      OP 1                            OP 2
    //      ----------------------------    ---------------
    // m1   T0: PC => addr                   
    // m2   T0: bus => data_reg              
    // m1   T1: data => ir, PC+1 => addr
    // m2   T1: bus => data_reg, decode
    // m1   T2: data_reg => alu (via DB)    T0: PC => addr
    // m2   T2: alu=>add                    T0: bus => data_reg
    // m1     : add=>a                      T1: data => ir, PC+1 => addr
    // m2                                   T1: bus => data_reg, decode
    // m1                                   T2: data_reg => alu (via DB)
    // m2                                   T2: alu=>add
    // m1                                       add=>a

    // timing state machine
    logic [5:0] Tstate, Tnext;
    logic T0next;           // signal that this is second to last cycle and next cycle is T0

    always_comb begin
        if( two_cycle && sync)  Tnext = 6'b101;         // T0+2
        else if (T0next)        Tnext = 6'b1;           // T0
        else                    Tnext = Tstate << 1;    // T++

        sync = Tstate[1];  // fetch opcode
    end

    localparam Treset = 6'b1;
    always_ff @(posedge clk_m1 ) begin
        if (i_rst) Tstate <= Trst;
        else Tstate <= Tnext;
    end

    // 6502 opcodes bits are organized as: aaabbbcc
    // https://www.masswerk.at/6502/6502_instruction_set.html#layout
    wire op_a = ir[7:5];
    wire op_b = ir[4:2];
    wire op_c = ir[0:1];

    // decode address mode, primarily organized by op_b;
    logic BRK;
    logic JSR;
    logic RTI;
    logic RTS;
    logic ABS;
    logic IMM;
    logic ZPG;
    logic IMM;
    logic IMPL;
    logic BR;
    logic JMP;
    logic XIND;
    logic INDY;
    logic ZPGXY;
    logic ABSXY;
    logic XYADDR;

    always @(*) begin
        BRK = 0;
        JSR = 0;
        RTI = 0;
        RTS = 0;
        ABS = 0;
        IMM = 0;
        ZPG = 0;
        IMM = 0;
        IMPL = 0;   // also inlcudes accumulator shifts
        BR = 0;
        JMP = 0;
        XIND = 0;
        INDY = 0;
        ZPGXY = 0;  // ZPGX or ZPGY based on XYADDR 
        ABSXY = 0;  // ABSX or ABSY based on XYADDR
        XYADDR = !(op_c[1] && (op_a == 3'b10?)); // default to ABSX, ZPGX, expect for on c=2,3 && a=4,5
        case(op_b)
            3'h0:   if ( !op_c[0] ) begin           
                        if ( op_a==3'h0 ) begin BRK = 1; IMPL=1; end
                        else if ( op_a==3'h1 ) begin JSR = 1; ABS=1; end
                        else if ( op_a==3'h2 ) begin RTI = 1; IMPL=1; end
                        else if ( op_a==3'h3 ) begin RTS = 1; IMPL=1; end
                        else IMM = 1;
                    end
                    else XIND = 1;
            3'h1:   ZPG = 1;
            3'h2:   if( op_c[0] ) IMM = 1;
                    else IMPL = 1;                      
            3'h3:   begin
                        // c=2, a=2,3 are JMP ABS and JMP IND.  All others are ABS)
                        if( op_c == 2'h0 && op_a == 3'b01? ) JMP = 1;
                        IND = JMP && op_a[0];
                        ABS = !IND;
                    end
            3'h4:   if( op_c == 2'h0) BR=1;
                    else INDY = 1;
            3'h5:   ZPGXY = 1;
            3'h6:   if( op_c[1]) begin
                        ABSXY=1;    
                        XYADDR=0;
                    end
                    else IMPL = 1;
            3'h7:   ABSXY=1;
        endcase
    end


    // decode opcode;
    // does not include control flow which have dedicated types above
    logic LD;      // ldx, ldy
    logic ST;      // stx, sty
    logic CP;      // cpx, cpy
    logic BIT;     // bit
    logic STACK;   // php, plp, pha, pla
    logic INC;     // inc, inx, iny
    logic DEC;     // dec, dex, dey
    logic MOV;     // tax, tay, tsx, txa, txs, tya
    logic ALU;     // ora, and, eor, adc, sta, lda, cmp, sbc
    logic SHIFT;   // asl, rol, lsr, ror
    logic CL;       // clc, cli, clv, cld
    logic SE;       // sec, sei, sed
    logic[1:0] SB_SRC;  // src reg for ALU, IMPL, ST, CP
    logic[1:0] SB_DST;  // dst reg for ALU, IMPL, LD
    logic NOP;
    logic RMW;

    always @(*) begin
        LD = 0;
        ST = 0;
        CP = 0;
        BIT = 0;
        STACK = 0;
        INC = 0;
        DEC = 0;
        MOV = 0;
        ALU = 0;
        SHIFT = 0;
        CL = 0;
        SE = 0;
        SB_DST = REGA;
        SB_SRC = REGA;
        RMW = 0;
        NOP = 0;

        casez(op_c)
            2'b00:  begin
                        // for a=7 operate on x, otherwise y
                        SB_DST = (op_a==3'h7) ? REGX : REGY; 
                        SB_SRC = SB_DST;
                        if (op_b==3'h0 || op_b[0]) begin            // b=0,1,3,5,7
                            if (op_a == 3'h1 && op_b==3'b0?1) BIT=1;// BIT
                            else if (op_a == 3'h4) ST=1;            // STY
                            else if (op_a == 3'h5) LD=1;            // LDY
                            else if (op_a == 3'b11?) CP=1;          // CPX, CPY
                        end
                        if(op_b == 3'h2) begin                      // b=2
                            if(op_a[2]) begin                       // a>=4
                                if (op_a[1:0] == 2'h0) DEC=1;       // DEY
                                else if (op_a[1:0] == 2'h1) begin   // TAY
                                    SB_SRC=REGA;
                                    MOV=1;
                                end else INC = 1;                   // INY, INX
                            end else begin                          // a<4
                                STACK = 1;                          // PHP, PLP, PHA, PLA
                            end
                        end
                        else if (op_b == 3'h4) BR = 1;              // branches
                        else if (op_b == 3'h6) begin                // mainly set/clear status
                            if (op_a == 3'h4) begin
                                MOV = 1;                            // TYA
                                SB_SRC = REGY;
                                SB_DST = REGA;
                            end else begin
                                CL = op_a[0] || op_a==3'h5;
                                SE = ~CL;
                            end
                        end
                    end
            2'b01:  ALU = 1;
            2'b1?:  begin
                        if (~op_a[2]) begin                             // a<4
                            SHIFT = 1;                                  // ASL,ROL, LSR, ROR
                            RMW = !(IMPL || IMM);                       // ops on mem are RMW
                        end else begin
                            SB_SRC = REGX;
                            SB_DST = REGX;
                            if (op_a == 3'h4) begin                     // STX, TXA, TXS
                                if( op_b == 3'b?10 ) begin              // b=2,6
                                    MOV = 1;
                                    if(op_b[2]) SB_DST = REGS;          // b=6, TXS
                                    if(op_b[2]) SB_DST = REGA;          // b=2, TXA
                                end else ST = 1;                        // STX
                            end
                            else if (op_a == 3'h5) begin                // LDX, TXS, TXS
                                if( op_b == 3'b?10 ) begin              // b=2,6
                                    MOV = 1;
                                    if(op_b[2]) SB_SRC = REGA;          // TAX
                                    else SB_SRC = REGS;                 // TSX
                                end else LD = 1;                        // LDX
                            end
                            else if (op_a == 3'h6) begin
                                DEC=1;                                  // DEC, DEX
                                RMW = !(IMPL || IMM);                   // ops on mem are RMW
                            end else if (op_a == 3'h7) begin
                                if(op_b == 3'h2) NOP = 1;               // NOP
                                else begin
                                    INC=1;                              // INC
                                    RMW = !(IMPL || IMM);               // ops on mem are RMW
                                end
                            end
                        end
                    end
        endcase
    end

    // ALU ops
    logic [3:0] exec_alu;
    always @(*) begin
        if(ALU) begin
            case(op_a):
                3'h0: exec_alu = ALU_ORA;
                3'h1: exec_alu = ALU_AND;
                3'h2: exec_alu = ALU_EOR;
                3'h3: exec_alu = ALU_ADC;
                3'h6: exec_alu = ALU_CMP;
                3'h7: exec_alu = ALU_SBC;
                default: exec_alu = ALU_NOP;
            endcase
        end else if (INC) exec_alu = ALU_INC;
        else if (DEC) exec_alu = ALU_DEC;
        else if (SHIFT) begin
            if (op_a[1]) exec_alu = ALU_SR;
            else         exec_alu = ALU_SL;
        end
        else if (BIT) exec_alu = ALU_BIT;
        else exec_alu = ALU_NOP;
    end

    // generate control signals
    db_src;
    sb_src;
    alu_srcA;
    // alu_srcB;
    adl_src;
    adh_src;
    store_reg;  // save X, Y, A, S, P
    alu_op;

    // higer level ctrl
    inc_pc;
    exec;
    logic [3:0] alu_ctl;

    logic [9:0] alu_ctl;

    parameter BUS_DATA    = 9'b0000000001;
    parameter BUS_SB      = 9'b0000000010;
    parameter BUS_ALU     = 9'b0000000100;
    parameter BUS_X       = 9'b0000001000;
    parameter BUS_Y       = 9'b0000010000;
    parameter BUS_A       = 9'b0000100000;
    parameter BUS_S       = 9'b0001000000;
    parameter BUS_P       = 9'b0010000000;
    parameter BUS_PCL     = 9'b0100000000;
    parameter BUS_PCH     = 9'b1000000000;


    db_src:     data, sb, alu, x, y, a, s, p, pcl, pch
    db_src:     din, sb, a, p, pcl, pch
    sb_src:     db,  adh, alu, x, y, a, s
    adl_src:    din, pcl, s, alu
    adh_src:    din, sb,  pch, 
    alu_src:    db, adl
    store_reg:  s, x, y, a, p, pcl, pch


    always @(*) begin

        // default signal path:
        db_src = DB_DIN;   // incoming data to db
        sb_src = SB_ADD;    // ALU out to sb
        alu_srcA = ALUA_DB; // db to ALU A input
        alu_op = ALU_INC;   // increment ALU_A
        update_reg = 0;      // dont update any registers

        // default: addr=pc
        adl_src = ADL_PC;
        adh_src = ADH_PC;
    
        if (Tstate[0]) begin // T0
            // 2nd to last cycle
            // ir still contains previous opcode
            exec = 1;

            // advance pc to fetch opcode on next cycle
            inc_pc = !IMPL;
        end
        if (Tstate[1]) begin // T1
            // Last cycle
            // ir still contains previous opcode
            update_reg = 1;

            // advance pc to fetch data on next cycle
            inc_pc = 1;
            pc_addr = 1;
        end
        if (Tstate[2]) begin // T2
            // first cycle for new opcode
            // ir is new opcode
            // db is pc+1 data

            if (ZPG || XIND || INDY) begin
                // fetch {00, db }
                db_adl = 1;             
                zp_adh = 1;
                T0next = ZPG;   // zpg op is ready to execute
                // compute db + index
                compute_offset = 1;
            end else if (ZPGXY) begin
                // maintain address
                adl2adl = 1;             
                adh2adh = 1;
            end else if (ABSXY) begin
                // fetch high addr
                inc_pc = 1;
                pc_addr = 1;
                // compute db + index
                compute_offset = 1;
            end            
        end
        if (Tstate[3]) begin
            if (XIND || INDY) begin
                // BAL in add
                // fetch {00, db+index }
                sb2adl = 1;
                zp2adh = 1;
            end else if (ZPGXY) begin
                // fetch {0, base + index }
                add2sb = 1;            
                sb2adl = 1;            
                zp2adh = 1;
                T0next = 1;   //ready to execute
            end else if (ABSXY) begin
                // fetch high addr
                inc_pc = 1;
                pc_addr = 1;
                // compute db + index
                compute_offset = 1;
            end            
        end

        if (compute_offset) begin
            x_sb = XYADDR;
            y_sb = !XYADDR;
            alu_ctl = ALU_ADD;
        end

        if (inc_adl) begin
            adl2add = 1;
            alu_ctl = ALU_INC;
        end

        if (exec) begin
            alu_ctl = exec_alu;            
            case(REG_SRC)
                REGX: x_sb=1; 
                REGY: y_sb=1; 
                REGA: a_sb=1; 
                REGS: s_sb=1; 
            endcase
        end

        if (save2reg) begin
            add_sb = 1;
            case(REG_DST)
                REGX: sb_x=1; 
                REGY: sb_y=1; 
                REGA: sb_a=1; 
                REGS: sb_s=1; 
            endcase
        end

        if (save2mem) begin
            add_sb = 1;
            sb_db = 1;
            db_dor = 1;
        end

        
    end



endmodule
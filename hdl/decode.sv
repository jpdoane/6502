module decode (
    input logic i_clk, i_rst,
    input  logic [7:0] opcode,
    input  logic [7:0] pstatus,

    output logic [5:0] initial_state,

    output logic    single_byte,  //single byte opcode
                    idx_XY, // index on X vs Y

    // memory access pattern
    output logic read, store,

    // bus ctl
    output logic [2:0] sb_src,   // sb bus source (registers)
    output logic [2:0] db_src,   // db bus source (data/addr)
    output logic [2:0] dst,  // location to store result

    // alu ctl
    output logic [2:0] alu_OP,
    output logic ai_inv,
    output logic bi_inv,
    output logic [1:0] carry_in, // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]

    // status ctl
    output logic [7:0] alu_mask, set_mask, clear_mask
    );

    logic [10:0] bus_ctl;         // {db_src, sb_src, res_src, res_dst}
    logic [13:0] alu_ctl;         // {alu_OP, ai_src, bi_src, carry_in, mask}
    assign {db_src, sb_src, dst} = bus_ctl;
    assign {alu_OP, ai_inv, bi_inv, carry_in, alu_mask} = alu_ctl;

    // decode initial state
    // https://www.masswerk.at/6502/6502_instruction_set.html#layout
    always @(*) begin
        casez(opcode)

            // op_b == 0
            8'b000_000_00:  initial_state = T2_BRK;             // BRK
            8'b001_000_00:  initial_state = T2_JSR;             // JSR
            8'b010_000_00:  initial_state = T2_RTI;             // RTI
            8'b011_000_00:  initial_state = T2_RTS;             // RTS
            8'b???_000_?1:  initial_state = T2_XIND;            // X,ind
            8'b1??_000_?0:  initial_state = T0_FETCH;           // LD/CP imm

            // op_b == 1
            8'b???_001_??:  initial_state = T2_ZPG;             // zpg

            // op_b == 2
            8'b0?0_010_00:  initial_state = T2_PUSH;           // php,pha
            8'b0?1_010_00:  initial_state = T2_POP;           // plp,pha
            8'b???_010_??:  initial_state = T0_FETCH;           // imm or impl

            // op_b == 3
            8'b01?_011_00:  initial_state = T2_JUMP;             // jmp abs, jmp ind
            8'b???_011_??:  initial_state = T2_ABS;             // abs

            // op_b == 4
            8'b???_100_00:  initial_state = take_branch ? T2_BRANCH : T0_FETCH; // conditional branch
            8'b???_100_?1:  initial_state = T2_INDY;            // alu ind,Y ops

            // op_b == 5
            8'b???_101_??:  initial_state = T2_ZPGXY;           // zpg X/Y

            // op_b == 6
            8'b???_110_?0:  initial_state = T0_FETCH;           // impl
            8'b???_110_?1:  initial_state = T2_ABSXY;           // abs, Y

            // op_b == 7
            8'b???_111_??:  initial_state = T2_ABSXY;           // abs,X/Y

            default: initial_state = T_JAM;
        endcase
    end

    // branch logic
    logic take_branch;
    always @(*) begin
        case(opcode[7:5]) //op_a
            3'h0:   take_branch = !pstatus[7]; // BPL
            3'h1:   take_branch = pstatus[7];  // BMI
            3'h2:   take_branch = !pstatus[6]; // BVC
            3'h3:   take_branch = pstatus[6];  // BVS
            3'h4:   take_branch = !pstatus[0]; // BCC
            3'h5:   take_branch = pstatus[0];  // BCS
            3'h6:   take_branch = !pstatus[1]; // BNE
            3'h7:   take_branch = pstatus[1];  // BEQ
        endcase
    end

    // single byte opcodes: b = 2 or 6 && c = 0 or 2
    assign single_byte = opcode ==? 8'b???_?10_?0;

    // X vs Y indexing
    assign idx_XY = (opcode ==? 8'b???_1?0_?? || opcode ==? 8'b???_1?1_10) ? IDX_X : IDX_Y;

    // set and clear masks
    always @(posedge i_clk ) begin
        if(i_rst) begin
            set_mask <= 8'h0;
            clear_mask <= 8'h0;
        end else begin
            set_mask <= 8'h0;
            clear_mask <= 8'h0;
            casez(opcode)
                8'b000_110_00: clear_mask <= FL_C; // CLC
                8'b010_110_00: clear_mask <= FL_I; // CLI
                8'b101_110_00: clear_mask <= FL_V; // CLV
                8'b110_110_00: clear_mask <= FL_D; // CLD
                8'b001_110_00: set_mask <= FL_C;   // SEC
                8'b011_110_00: set_mask <= FL_I;   // SEI
                8'b111_110_00: set_mask <= FL_D;   // SED
            endcase
        end
    end

    // decode bus routing
    // https://www.masswerk.at/6502/6502_instruction_set.html#layout


    always @(posedge i_clk ) begin
        if(i_rst) begin
            bus_ctl <= {DB_Z, REG_Z, REG_Z};
        end else begin
            casez(opcode)

                //op_c==0
                8'b0??_000_00:  bus_ctl <= {DB_Z, REG_Z, REG_Z};       // control flow and special ops

                8'b000_010_00:  bus_ctl <= {DB_P, REG_Z, REG_DATA};     // PHP 
                8'b010_010_00:  bus_ctl <= {DB_Z, REG_A, REG_DATA};     // PHA
                8'b001_010_00:  bus_ctl <= {DB_DATA, REG_Z, REG_P};     // PLP
                8'b011_010_00:  bus_ctl <= {DB_DATA, REG_Z, REG_A};     // PLA
                8'b101_010_00:  bus_ctl <= {DB_Z, REG_A, REG_Y};        // TAY
                8'b111_010_00:  bus_ctl <= {DB_Z, REG_X, REG_X};       // INX
                8'b1?0_010_00:  bus_ctl <= {DB_Z, REG_Y, REG_Y};       // DEY, INY

                8'b001_0?1_00:  bus_ctl <= {DB_DATA, REG_A, REG_Z};    // BIT

                8'b100_110_00:  bus_ctl <= {DB_Z, REG_Y, REG_A};        // TYA
                8'b???_1?0_00:  bus_ctl <= {DB_DATA, REG_Z, REG_Z};    // branch & mask ops (no bus)

                8'b100_??1_00:  bus_ctl <= {DB_Z, REG_Y, REG_DATA};  // STY
                8'b101_???_00:  bus_ctl <= {DB_DATA, REG_Z, REG_Y};     // LDY
                8'b110_0??_00:  bus_ctl <= {DB_DATA, REG_Y, REG_Z};       // CPY
                8'b111_0??_00:  bus_ctl <= {DB_DATA, REG_X, REG_Z};       // CPX

                //op_c==1, accumulator ops
                8'b100_???_?1:  bus_ctl <= {DB_Z, REG_A, REG_DATA};  // STA
                8'b101_???_?1:  bus_ctl <= {DB_DATA, REG_Z, REG_A};     // LDA
                8'b110_???_?1:  bus_ctl <= {DB_DATA, REG_A, REG_Z};    // CMP
                8'b???_???_?1:  bus_ctl <= {DB_DATA, REG_A, REG_A};    // other ALU

                //op_c==2, mostly unary ops and LD/ST
                8'b100_??1_10:  bus_ctl <= {DB_Z, REG_X, REG_DATA};     // STX
                8'b100_010_10:  bus_ctl <= {DB_Z, REG_X, REG_A};        // TXA
                8'b100_110_10:  bus_ctl <= {DB_Z, REG_X, REG_S};        // TXS
                8'b101_010_10:  bus_ctl <= {DB_Z, REG_A, REG_X};        // TAX
                8'b101_110_10:  bus_ctl <= {DB_Z, REG_S, REG_X};        // TSX
                8'b101_???_10:  bus_ctl <= {DB_DATA, REG_Z, REG_X};     // LDX
                8'b110_010_10:  bus_ctl <= {DB_Z, REG_X, REG_X};       // DEX
                8'b111_010_10:  bus_ctl <= {DB_Z, REG_Z, REG_Z};       // NOP

                8'b???_010_10:  bus_ctl <= {DB_Z, REG_A, REG_A};       // accumulator op
                8'b???_???_10:  bus_ctl <= {DB_DATA, REG_Z, REG_DATA}; // rmw

                default:        bus_ctl <= {DB_Z, REG_Z, REG_Z};
            endcase
        end
    end

    // decode alu
    always @(posedge i_clk ) begin
        if(i_rst) begin
            alu_ctl <= {ALU_ADD, 2'b00, CARRY_Z, FL_NONE};
        end else begin
            casez(opcode)

                //NOP
                8'b111_010_10:  alu_ctl <= {ALU_NOP, 2'b00, CARRY_Z, FL_NONE};                   // NOP

                // bit, inc, dec
                8'b001_0?1_00:  alu_ctl <= {ALU_BIT, 2'b00, CARRY_Z, FL_N | FL_V | FL_Z};        // BIT
                8'b100_010_00:  alu_ctl <= {ALU_ADD, 2'b01, CARRY_Z, FL_N | FL_Z};               // DEY
                8'b11?_010_00:  alu_ctl <= {ALU_ADD, 2'b00, CARRY_P, FL_N | FL_Z};               // INY, INX
                8'b110_???_10:  alu_ctl <= {ALU_ADD, 2'b10, CARRY_Z, FL_N | FL_Z};               // DEC
                8'b111_???_10:  alu_ctl <= {ALU_ADD, 2'b00, CARRY_P, FL_N | FL_Z};               // INC

                // compare
                8'b11?_0??_00,
                8'b110_???_?1:  alu_ctl <= {ALU_ADD, 2'b01, CARRY_P, FL_N | FL_Z | FL_C};        // CMP, CPX, CPY

                // alu artihmetic
                8'b000_???_?1:  alu_ctl <= {ALU_OR, 2'b00, CARRY_Z, FL_N | FL_Z};                // ORA
                8'b001_???_?1:  alu_ctl <= {ALU_AND, 2'b00, CARRY_Z, FL_N | FL_Z};               // AND
                8'b010_???_?1:  alu_ctl <= {ALU_XOR, 2'b00, CARRY_Z, FL_N | FL_Z};               // EOR
                8'b011_???_?1:  alu_ctl <= {ALU_ADD, 2'b00, CARRY_C, FL_N | FL_V | FL_Z | FL_C}; // ADC
                8'b111_???_?1:  alu_ctl <= {ALU_ADD, 2'b01, CARRY_C, FL_N | FL_V | FL_Z | FL_C}; // SBC

                //shift/rot
                8'b000_???_10:  alu_ctl <= {ALU_SL, 2'b00, CARRY_Z, FL_N | FL_Z | FL_C };        // ASL
                8'b001_???_10:  alu_ctl <= {ALU_SL, 2'b00, CARRY_C, FL_N | FL_Z | FL_C };        // ROL
                8'b010_???_10:  alu_ctl <= {ALU_SR, 2'b00, CARRY_Z, FL_N | FL_Z | FL_C };        // LSR
                8'b110_???_10:  alu_ctl <= {ALU_SR, 2'b00, CARRY_C, FL_N | FL_Z | FL_C };        // ROR


                // transfers, loads, pulls (pass through ALU to set the status flags)
                8'b100_110_00,  //TAY
                8'b101_010_00,  //TAY
                8'b10?_?10_10,  //TAX,TXA,TXS,TSX
                8'b101_000_?0,  //LD
                8'b101_??1_?0,  //LD
                8'b101_???_01,  //LD
                8'b0?1_010_00:  alu_ctl <= {ALU_OR, 2'b00, CARRY_Z, FL_N | FL_Z}; //PLP, PLA

                default:        alu_ctl <= {ALU_NOP, 2'b00, CARRY_Z, FL_NONE};
            endcase
        end
    end

    assign read = db_src == DB_DATA;
    assign store = dst == REG_DATA;


endmodule

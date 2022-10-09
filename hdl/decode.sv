module decode (
    input logic i_clk, i_rst,
    input  logic [7:0] opcode,
    input  logic [7:0] pstatus,

    output logic [4:0] initial_state,

    output logic    single_byte,  //single byte opcode
                    idx_XY, // index on X(1) or Y(0)

    // memory access pattern
    output logic read, load, store, rmw,

    // bus ctl
    output logic [2:0] sb_src,   // sb bus source (registers)
    output logic [2:0] db_src,   // db bus source (data/addr)
    output logic [1:0] res_src,  // source of result (alu, sb, db)
    output logic [2:0] res_dst,  // location to store result (registers, mem)

    // alu ctl
    output logic [2:0] alu_OP,
    output logic [1:0] ai_src,
    output logic [1:0] bi_src,
    output logic [1:0] carry_in, // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]

    // status ctl
    output logic [7:0] alu_mask, set_mask, clear_mask
    );

    // decode opcode
    // rather than a huge mux on full opcode, utilize layout patterns
    // https://www.masswerk.at/6502/6502_instruction_set.html#layout
    logic [2:0] op_a;
    logic [2:0] op_b;
    logic [1:0] op_c;
    assign {op_a, op_b, op_c} = opcode;

    // branch logic
    logic take_branch;
    always @(*) begin
        case(op_a)
            3'h0:   take_branch = !pstatus[7]; // BPL
            3'h1:   take_branch = pstatus[7]; // BMI
            3'h2:   take_branch = !pstatus[6]; // BVC
            3'h3:   take_branch = pstatus[6]; // BVS
            3'h4:   take_branch = !pstatus[0]; // BCC
            3'h5:   take_branch = pstatus[0]; // BCS
            3'h6:   take_branch = !pstatus[1]; // BNE
            3'h7:   take_branch = pstatus[1]; // BEQ
        endcase
    end

    // decode address mode, which determines initialtake_branch state
    // primarily differentiated by op_b
    always @(*) begin
        single_byte = 0;
        idx_XY = 1;
        initial_state = T_JAM;
        case(op_b)
            3'h0:   if (op_c[0] == 1) initial_state = T2_XIND;           // X,ind
                    else if (op_a[2] == 0) initial_state = T_JAM;        // BRK, JSR, RTI, RTS
                    else initial_state = T0_FETCH;                             // LD/CP imm
            3'h1:   initial_state = T2_ZPG;                              // all zpg
            3'h2:   begin
                        initial_state = (op_c == 0 && !op_a[2]) ? T2_STACK : // c=0, a<4 stack ops
                                                                 T0_FETCH ;  // others imm or impl 
                        single_byte = !op_c[0];                                     // c=0,2 are single byte
                    end
            3'h3:   if (op_a[2:1] == 'b01 && op_c == 0) initial_state = T2_JMP;
                    else if (op_a[2:1] == 'b01 && op_c == 'b00) initial_state = T_JAM;      // jump abs,ind
                    else initial_state = T2_ABS;                         // abs ops
            3'h4:   if (op_c == 'b00)
                        initial_state =  take_branch ? T2_BRANCH : T0_FETCH; // branches
                    else begin
                        idx_XY = 0;
                        initial_state = T2_INDY;                        // alu ind,Y ops
                    end
            3'h5:   begin
                        initial_state = T2_ZPGXY;                        // zpg,X/Y
                        idx_XY = !(op_c==2 && op_a[2:1]=='b10);
                    end
            3'h6:   if (op_c[0] == 0) begin
                        single_byte = 1;
                        initial_state = T0_FETCH;               // impl
                    end 
                    else begin
                        initial_state = T2_ABSXY;                        // abs,Y
                        idx_XY = 0;
                    end
            3'h7:   begin
                        initial_state = T2_ABSXY;                        // abs,X/Y
                        idx_XY = !(op_c==2 && op_a[2:1]=='b10);
                    end
        endcase
    end


    always @(posedge i_clk ) begin
        if(i_rst) begin
            db_src <= DB_DATA;
            sb_src <= REG_A;
            ai_src <= ADD_BUS;
            bi_src <= ADD_BUS;
            res_src <= RES_ADD;
            res_dst <= REG_A;

            alu_OP <= ALU_ADD;
            alu_mask <= 0;        // default do not update status flag
            carry_in <= 2'b00;   // default carry in <= 0

            set_mask <= 8'h0;
            clear_mask <= 8'h0;
        end else begin

            //default alu behavor: A + mem -> A
            db_src <= DB_DATA;
            sb_src <= REG_A;
            ai_src <= ADD_BUS;
            bi_src <= ADD_BUS;
            res_src <= RES_ADD;
            res_dst <= REG_A;

            alu_OP <= ALU_ADD;
            alu_mask <= 0;        // default do not update status flag
            carry_in <= 2'b00;   // default carry in <= 0

            set_mask <= 8'h0;
            clear_mask <= 8'h0;

            case(op_c)
                0: begin
                    //primarily control flow, special, and load/store/tranfer
                    res_dst <= REG_Z; // default no register update

                    if(op_b==4) begin end// branch op
                    else if(op_b==6) begin   // set/clear flags
                        case(op_a)
                            0: clear_mask[0] <= 1; // CLC
                            1: set_mask[0] <= 1;   // SEC
                            2: clear_mask[2] <= 1; // CLI
                            3: set_mask[2] <= 1; // SEI
                            4:  begin //TYA
                                sb_src <= REG_Y;
                                res_src <= RES_SB;
                                res_dst <= REG_A;
                                end
                            5: clear_mask[6] <= 1; // CLV
                            6: clear_mask[3] <= 1; // CLD
                            7: set_mask[3] <= 1; // SED
                        endcase
                    end
                    else if(op_a[2] == 0) begin // a=0:3, b != 6
                        if (op_b == 3'h2) begin //Stack push/pull
                            res_src <= RES_DB;
                            if (op_a[0]) begin
                                // pull
                                res_src <= RES_DB;
                                res_dst <= op_a[1] ? REG_A : REG_P;
                            end else begin
                                // push
                                db_src <= op_a[1] ? DB_A : DB_P;
                                res_dst <= REG_DATA;
                            end
                        end if (op_a[1:0] == 2'b01 && op_b[0]) begin// BIT
                            // sb_src <= REG_A;
                            // db_src <= DB_DATA;
                            alu_OP <= ALU_BIT;
                            res_dst <= REG_Z;
                            alu_mask <= 8'b11000010;
                        end
                        // control flow and special ops
                        // TODO....
                    end
                    else if(op_b == 3'b010) begin // a=4:7, b=2
                        //DEY, TAY, INY, INX
                        if (op_a == 5) begin //TAY
                            res_dst <= REG_Y;
                            sb_src <= REG_A;
                            res_src <= RES_SB;
                        end else begin // DEY, INY, INX
                            sb_src <= REG_Y;
                            bi_src <= ADD_Z;
                            res_src <= RES_ADD;
                            res_dst <= REG_Y;
                            alu_mask <= 8'b10000010; // status mask for inc/dec
                            if(op_a==4) begin //DEY
                                bi_src <= ADD_N; //bi <= -1
                                carry_in <= 2'b00; // C=0 
                            end else if(op_a==6) begin //INY
                                carry_in <= 2'b01; // C=1
                            end else if(op_a==7) begin //INX
                                sb_src <= REG_X;
                                res_dst <= REG_X;
                                carry_in <= 2'b01; // C=1
                            end
                        end
                    end
                    // remaining options are restriced to a>=4, b=0,1,3,5,7
                    else if(op_a==4) begin// STY
                        sb_src <= REG_Y;
                        res_src <= RES_SB;
                        res_dst <= REG_DATA;
                    end
                    else if(op_a == 5) begin //LDY
                        res_src <= RES_DB;
                        res_dst <= REG_Y;
                    end
                    else begin // op_a=6-7: CPX/CPY
                        // alu_OP <= ALU_ADD;    //already default
                        carry_in <= 2'b01; // C=1 (equivalent to borrow=0)
                        bi_src <= ADD_INVBUS; // ai-bi
                        sb_src <= op_a[0] ? REG_X : REG_Y;
                        res_dst <= REG_Z; //dont store result, just status mask
                        alu_mask <= 8'b10000011;
                    end
                end
            1:  begin
                    // accumulator operations
                    // already default:
                    // sb_src <= REG_A;
                    // db_src <= DB_DATA;
                    // res_src <= RES_ADD;
                    // res_dst <= REG_A;
                    case(op_a)
                        0:  begin   //ORA
                            alu_OP <= ALU_OR;
                            // carry_in <= 2'b00;
                            alu_mask <= 8'b10000010;
                            end
                        1:  begin   //AND
                            alu_OP <= ALU_AND;
                            // carry_in <= 2'b00;
                            alu_mask <= 8'b10000010;
                            end
                        2:  begin   //EOR
                            alu_OP <= ALU_XOR;
                            // carry_in <= 2'b00;
                            alu_mask <= 8'b10000010;
                            end
                        3:  begin   //ADC
                            // alu_OP <= ALU_ADD;
                            carry_in <= 2'b10;   // C=P[C]
                            alu_mask <= 8'b11000011;
                            end
                        4:  begin //STA
                            // sb_src <= REG_A;
                            res_src <= RES_SB;
                            res_dst <= REG_DATA;
                            end
                        5:  begin //LDA
                            res_src <= RES_DB;
                            end
                        6:  begin // CMP;
                            // alu_OP <= ALU_ADD;
                            carry_in <= 2'b01; // C=1 (borrow=0)
                            bi_src <= ADD_INVBUS;        // negate DB
                            alu_mask <= 8'b10000011;
                            res_dst <= REG_Z; //dont update registers
                            end
                        7:  begin // SBC;
                            // alu_OP <= ALU_ADD;
                            carry_in <= 2'b10; // C=P[C]
                            bi_src <= ADD_INVBUS;        // negate DB
                            alu_mask <= 8'b11000011;
                            end
                    endcase
                end
            2:  begin
                    // mostly unary operations (shift, rot, inc, dec)
                    if (op_b == 2 || op_b == 4) begin
                        // defaults:
                        // sb_src <= REG_A;
                        // res_src <= RES_ADD;
                        // res_dst <= REG_A;
                        bi_src <= ADD_Z; // op(reg,0)
                    end else begin
                        // defaults
                        // db_src <= DB_DATA;
                        // res_src <= RES_ADD;
                        ai_src <= ADD_Z; // op(0,MEM)
                        res_dst <= REG_DATA;
                    end
                    
                    case(op_a)
                        0:  begin // ASL
                            alu_OP <= ALU_SL;
                            // carry_in <= 2'b00;    //already default
                            alu_mask <= 8'b10000011;
                            end
                        1:  begin // ROL
                            alu_OP <= ALU_SL;
                            carry_in <= 2'b10;       // C <= P[C]
                            alu_mask <=8'b10000011;
                            end
                        2:  begin // LSR
                            alu_OP <= ALU_SR;
                            // carry_in <= 2'b00;
                            alu_mask <= 8'b00000011;
                            end
                        3:  begin // ROR
                            alu_OP <= ALU_SR;
                            carry_in <= 2'b10; // C <= P[C]
                            alu_mask <= 8'b10000011;
                            end
                        4:  begin // copy from X: STX, TXA, TXY
                            sb_src <= REG_X;
                            res_src <= RES_SB; 
                            case(op_b)
                                2:          res_dst <= REG_A;
                                6:          res_dst <= REG_S;
                                default:    res_dst <= REG_DATA;
                            endcase
                            end
                        5:  begin // copy to X: LDX, TAX, TSX
                            res_dst <= REG_X;
                            res_src <= (op_b == 2 || op_b == 6) ? RES_SB : RES_DB; 
                            sb_src <= (op_b == 6) ? REG_S : REG_A;
                            end
                        6:  begin // DEC
                            // alu_OP <= ALU_ADD;            //already default
                            // carry_in <= 2'b00;        //already default
                            if (op_b == 2)  begin
                                            //DEX
                                            sb_src <= REG_X;
                                            res_dst <= REG_X; 
                                            bi_src <= ADD_N; // op(reg,-1)
                                            end
                            else            ai_src <= ADD_N;  // op(-1,MEM)
                            alu_mask <= 8'b10000010;
                            end
                        7:  begin // INC, NOP
                            // alu_OP <= ALU_ADD;
                            carry_in <= 2'b01; // C=1
                            alu_mask <= 8'b10000010;
                            if (op_b==2) begin
                                alu_mask <= 0;
                                res_dst <= REG_Z;
                            end
                            end
                    endcase
                end
            endcase
        end
    end


    // memory modes
    // determined by: res_src and res_dst
    always @(*) begin
        read = 0;
        load = 0; //also stack pull
        store = 0; //also stack push
        rmw = 0;
        if(res_dst == REG_DATA)
            if(res_src == RES_ADD) rmw = 1;
            else store = 1;                   
        else
            if(res_src == RES_DB) load = 1;
            else read = 1;
    end    





endmodule

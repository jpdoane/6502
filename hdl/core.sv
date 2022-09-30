`include "defs.svi"


module core #(
    parameter T_BOOT_ADDR=16'h0)
    (
    input  logic i_clk, i_rst,
    input  logic [7:0] i_data,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,

    output logic [15:0] addr,
    output logic [7:0] dor,
    output logic RW
    );

    // busses
    // db: data bus: typically memory read, but can be addr_lo or zero
    // sb: secondry bus: read from registers (a,x,y,s, zero)
    // res: result bus: update register (a,x,y,s,d [mem write], z[disabled])
    logic [7:0] db, sb, res;

    // registers
    logic [15:0] pc;
    logic [7:0] a, s, x, y, p;
    logic [7:0] radl, radh;
    logic [7:0] ir;
    logic [7:0] add;
    logic IPC;

    logic [3:0] res_dst;
    always @(posedge i_clk ) begin
        if (i_rst) begin
            a <= 0;
            x <= 0;
            y <= 0;
            s <= 0;
            dor <= 0;
            radh <= 0;
            radl <= 0;
            pc <= T_BOOT_ADDR;
            //p updated elsewhere...
        end else begin
            a <= a;
            x <= x;
            y <= y;
            s <= s;

            radh <= addr[15:8];
            radl <= addr[7:0];
            pc <= IPC ? pc+1 : pc;
            RW <= 1;

            case(res_dst)
                REG_A: a <= res;
                REG_X: x <= res;
                REG_Y: y <= res;
                REG_S: s <= res;
                REG_D:  begin
                        // critical that address is also set properly!!
                        dor <= res;
                        RW <= 0;
                        end
            endcase
        end
    end
    
    // bus source select
    logic [1:0] db_src;
    logic [2:0] sb_src;
    logic [1:0] res_src;
    always @(*) begin
        case(db_src)
            DB_DL:      db = i_data;
            DB_AL:      db = addr[7:0];
            default:    db = 8'h0;
        endcase
        case(sb_src)
            REG_A: sb = a;
            REG_X: sb = x;
            REG_Y: sb = y;
            REG_S: sb = s;
            default: sb = 8'h0;
        endcase
        case(res_src)
            RES_DB: res = db;
            RES_SB: res = sb;
            RES_ADD: res = add;
            default: res = 8'h0;
        endcase
    end


    //alu
    logic [7:0] ai, bi, alu_out;
    logic ci;
    logic aluN, aluZ, aluC, aluV;

    logic [2:0] ai_src;

    logic [2:0] alu_OP;
    logic alu_ZEROAI;   // force ai=0
    logic alu_ZEROBI;   // force bi=0
    logic alu_INVAI;    // invert ai
    logic alu_INVBI;    // invert bi
    logic alu_AORB;     // operate on (AI|BI)
                    // (unary op from either port and self-add for left-shifts)
    logic [1:0] alu_carry_in; // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]
    logic [5:0] alu_st_mask;  // mask to update status flags: NZCIDV


    always @(*) begin
        ai = alu_INVAI ? ~sb : sb;
        bi = alu_INVBI ? ~db : db;

        if (alu_AORB) begin
            ai |= bi;
            bi = ai;
        end

        ci = alu_carry_in[1] ? p[0] : alu_carry_in[0];

        case(alu_OP)
            ALU_ADD:    {aluC, alu_out} = ai + bi + ci;
            ALU_AND:    alu_out = ai & bi;
            ALU_OR:     alu_out = ai | bi;
            ALU_XOR:    alu_out = ai ^ bi;
            ALU_SR:     {alu_out, aluC} = {ci, ai};
        endcase

        aluZ = ~|alu_out;
        aluN = alu_out[7];
        aluV = ai[7] ^ bi[7] ^ aluC ^ aluN; 
    end

    // register alu and p
    logic carry_out; // also register carry out
    logic[7:0] set_mask, clear_mask;
    always @(posedge i_clk ) begin
        if (i_rst) begin
            add <= 8'b0;
            p <= 8'b0;
            carry_out <= 0;
        end else begin
            add <= alu_out;

            p[7] <= clear_mask[7] ? 0 : set_mask[7] ? 1 : alu_st_mask[5] ? aluN : p[7];
            p[6] <= clear_mask[6] ? 0 : set_mask[6] ? 1 : alu_st_mask[0] ? aluV : p[6];
            p[5] <= clear_mask[5] ? 0 : set_mask[5] ? 1 : p[5];
            p[4] <= clear_mask[4] ? 0 : set_mask[4] ? 1 : p[4];
            p[3] <= clear_mask[3] ? 0 : set_mask[3] ? 1 : p[3];
            p[2] <= clear_mask[2] ? 0 : set_mask[2] ? 1 : p[2];
            p[1] <= clear_mask[1] ? 0 : set_mask[1] ? 1 : alu_st_mask[4] ? aluZ : p[1];
            p[0] <= clear_mask[0] ? 0 : set_mask[0] ? 1 : alu_st_mask[3] ? aluC : p[0];

            // p[7] <= !clear_mask[7] && (p[7] || set_mask[7] || (aluN && alu_st_mask[5]));
            // p[6] <= !clear_mask[6] && (p[6] || set_mask[6] || (aluV && alu_st_mask[0]));
            // p[5] <= !clear_mask[5] && (p[5] || set_mask[5]);
            // p[4] <= !clear_mask[4] && (p[4] || set_mask[4]);
            // p[3] <= !clear_mask[3] && (p[3] || set_mask[3]);
            // p[2] <= !clear_mask[2] && (p[2] || set_mask[2]);
            // p[1] <= !clear_mask[1] && (p[1] || set_mask[1] || (aluZ && alu_st_mask[4]));
            // p[0] <= !clear_mask[0] && (p[0] || set_mask[0] || (aluC && alu_st_mask[3]));
            carry_out <= aluC;
        end
    end


    // state
    logic [4:0] state, initial_state;
    logic state_skip;

    // ir fetch
    assign SYNC = state == T0;
    always @(posedge i_clk ) begin
        if (i_rst)          ir <= 8'hea; //NOP
        else if (state==T1) ir <= i_data;
    end
    logic [7:0] current_op;
    assign current_op = (state==T1) ? i_data : ir;

    // decode opcode
    // rather than a huge mux on full opcode, utilize layout patterns
    // https://www.masswerk.at/6502/6502_instruction_set.html#layout
    logic [2:0] op_a;
    logic [2:0] op_b;
    logic [1:0] op_c;
    assign {op_a, op_b, op_c} = current_op;

    // decode address mode, which determines initial state
    // primarily differentiated by op_b
    logic op_SB;
    logic idx_XY;           // select index X(1) or Y(0) for abs,X/Y and zpg,X/Y ops
    always @(*) begin
        op_SB = 0;
        idx_XY = 1;
        initial_state = T_JAM;
        case(op_b)
            3'h0:   if (op_c[0] == 1) initial_state = T2_XIND;           // X,ind
                    else if (op_a[2] == 0) initial_state = T_JAM;        // BRK, JSR, RTI, RTS
                    else initial_state = T0;                             // LD/CP imm
            3'h1:   initial_state = T2_ZPG;                              // all zpg
            3'h2:   begin
                        initial_state = T0;                              // all imm or impl
                        op_SB = !op_c[0];                                // impl are single byte
                    end
            3'h3:   if (op_a[2:1] == 'b01 && op_c == 'b00) initial_state = T_JAM;      // jump abs,ind
                    else initial_state = T2_ABS;                         // abs ops
            3'h4:   if (op_c == 'b00)   initial_state = T_JAM;            // branches
                    else begin
                        idx_XY = 0;
                        initial_state = T2_INDY;                        // alu ind,Y ops
                    end
            3'h5:   begin
                        initial_state = T2_ZPGXY;                        // zpg,X/Y
                        idx_XY = !(op_c==2 && op_a[2:1]=='b10);
                    end
            3'h6:   if (op_c[0] == 0) begin
                        op_SB = 1;
                        initial_state = T0;               // impl
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

    // decode opcode type and data access pattern
    // primarily differentiated by op_a and op_c
    logic [3:0] ex_sb_src;   // sb bus source (registers)
    logic [2:0] ex_db_src;   // db bus source (data/addr)
    logic [2:0] ex_res_src;  // source of result (alu, sb, db)
    logic [2:0] ex_res_dst;  // location to store result (registers, mem)

    logic [2:0] ex_alu_OP;
    logic ex_alu_INVAI;    // invert ai (ai=ff if alu_ZEROAI)
    logic ex_alu_INVBI;    // invert bi (ai=ff if alu_ZEROBI)
    logic ex_alu_AORB;     // operate on (AI|BI)
                    // (unary op from either port and self-add for left-shifts)
    logic [1:0] ex_alu_carry_in; // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]
    logic [5:0] ex_alu_st_mask;  // mask to update status flags: NZCIDV

    logic branch;

    always @(posedge i_clk ) begin
        //default alu behavor: A + mem -> A
        ex_db_src <= DB_DL;
        ex_sb_src <= REG_A;
        ex_res_src <= RES_ADD;
        ex_res_dst <= REG_A;

        ex_alu_OP <= ALU_ADD;
        ex_alu_INVAI <= 0;
        ex_alu_INVBI <= 0;
        ex_alu_AORB <= 0;
        ex_alu_st_mask <= 0;        // default do not update status flag
        ex_alu_carry_in <= 2'b00;   // default carry in <= 0

        set_mask <= 8'h0;
        clear_mask <= 8'h0;
        branch = 0;

        case(op_c)
            0: begin
                //primarily control flow, special, and load/store/tranfer
                ex_res_dst <= REG_Z; // default no register update

                if(op_b==4) branch=1;// branch op
                else if(op_b==6) begin   // set/clear flags
                    case(op_a)
                        0: clear_mask[0] <= 1; // CLC
                        1: set_mask[0] <= 1;   // SEC
                        2: clear_mask[2] <= 1; // CLI
                        3: set_mask[2] <= 1; // SEI
                        4:  begin //TYA
                            ex_sb_src <= REG_Y;
                            ex_res_src <= RES_SB;
                            ex_res_dst <= REG_A;
                            end
                        5: clear_mask[6] <= 1; // CLV
                        6: clear_mask[3] <= 1; // CLD
                        7: set_mask[3] <= 1; // SED
                    endcase
                end
                else if(op_a[2] == 0) begin // a=0:3, b != 6
                    // control flow and special ops
                    // TODO....
                end
                else if(op_b == 3'b010) begin // a=4:7, b=2
                    //DEY, TAY, INY, INX
                    if (op_a == 5) begin //TAY
                        ex_res_dst <= REG_Y;
                        ex_sb_src <= REG_A;
                        ex_res_src <= RES_SB;
                    end else begin // DEY, INY, INX
                        ex_sb_src <= REG_Y;
                        ex_db_src <= DB_Z;
                        ex_res_src <= RES_ADD;
                        ex_res_dst <= REG_Y;
                        ex_alu_st_mask <= 6'b110000; // status mask for inc/dec
                        if(op_a==4) begin //DEY
                            ex_alu_INVBI <= 1; //bi <= -1
                            ex_alu_carry_in <= 2'b00; // C=0 
                        end else if(op_a==6) begin //INY
                            ex_alu_carry_in <= 2'b01; // C=1
                        end else if(op_a==7) begin //INX
                            ex_sb_src <= REG_X;
                            ex_res_dst <= REG_X;
                            ex_alu_carry_in <= 2'b01; // C=1
                        end
                    end
                end
                // remaining options are restriced to a>=4, b=0,1,3,5,7
                else if(op_a==4) begin// STY
                    ex_sb_src <= REG_Y;
                    ex_res_src <= RES_SB;
                    ex_res_dst <= REG_D;
                end
                else if(op_a == 5) begin //LDY
                    ex_res_src <= RES_DB;
                    ex_res_dst <= REG_Y;
                end
                else begin // op_a=6-7: CPX/CPY
                    // ex_alu_OP <= ALU_ADD;    //already default
                    ex_alu_carry_in <= 2'b01; // C=1 (equivalent to borrow=0)
                    ex_alu_INVBI <= 1; // ai-bi
                    ex_sb_src <= op_a[0] ? REG_X : REG_Y;
                    ex_res_dst <= REG_Z; //dont store result, just status mask
                    ex_alu_st_mask <= 6'b111000;
                end
            end
        1:  begin
                // accumulator operations
                // already default:
                // ex_sb_src <= REG_A;
                // ex_db_src <= DB_DL;
                // ex_res_src <= RES_ADD;
                // ex_res_dst <= REG_A;
                case(op_a)
                    0:  begin   //ORA
                        ex_alu_OP <= ALU_OR;
                        // ex_alu_carry_in <= 2'b00;
                        ex_alu_st_mask <= 6'b110000;
                        end
                    1:  begin   //AND
                        ex_alu_OP <= ALU_AND;
                        // ex_alu_carry_in <= 2'b00;
                        ex_alu_st_mask <= 6'b110000;
                        end
                    2:  begin   //EOR
                        ex_alu_OP <= ALU_XOR;
                        // ex_alu_carry_in <= 2'b00;
                        ex_alu_st_mask <= 6'b110000;
                        end
                    3:  begin   //ADC
                        // ex_alu_OP <= ALU_ADD;
                        ex_alu_carry_in <= 2'b10;   // C=P[C]
                        ex_alu_st_mask <= 6'b111001;
                        end
                    4:  begin //STA
                        // ex_sb_src <= REG_A;
                        ex_res_src <= RES_SB;
                        ex_res_dst <= REG_D;
                        end
                    5:  begin //LDA
                        ex_res_src <= RES_DB;
                        end
                    6:  begin // CMP;
                        // ex_alu_OP <= ALU_ADD;
                        ex_alu_carry_in <= 2'b01; // C=1 (borrow=0)
                        ex_alu_INVBI <= 1;        // negate DB
                        ex_alu_st_mask <= 6'b111000;
                        ex_res_dst <= REG_Z; //dont update registers
                        end
                    7:  begin // SBC;
                        // ex_alu_OP <= ALU_ADD;
                        ex_alu_carry_in <= 2'b10; // C=P[C]
                        ex_alu_INVBI <= 1;        // negate DB
                        ex_alu_st_mask <= 6'b111001;
                        end
                endcase
            end
        2:  begin
                // mostly unary operations (shift, rot, inc, dec)
                if (op_b == 2 && op_b == 4) begin
                    // defaults:
                    // ex_sb_src <= REG_A;
                    // ex_res_src <= RES_ADD;
                    // ex_res_dst <= REG_A;
                    ex_db_src <= DB_Z; // op(reg,0)
                end else begin
                    // defaults
                    // ex_db_src <= DB_DL;
                    // ex_res_src <= RES_ADD;
                    ex_sb_src <= REG_Z; // op(0,MEM)
                    ex_res_dst <= REG_D;
                end
                
                // for shift ops, use op(AI | BI)
                ex_alu_AORB <= op_a[2] == 0;      

                case(op_a)
                    0:  begin // ASL: A+A+0
                        // ex_alu_OP <= ALU_ADD;        //already default
                        // ex_alu_carry_in <= 2'b00;    //already default
                        ex_alu_st_mask <= 6'b111000;
                        end
                    1:  begin // ROL: A+A+C
                        // ex_alu_OP <= ALU_ADD;        //already default
                        ex_alu_carry_in <= 2'b10;       // C <= P[C]
                        ex_alu_st_mask <= 6'b111000;
                        end
                    2:  begin // LSR
                        ex_alu_OP <= ALU_SR;
                        // ex_alu_carry_in <= 2'b00;
                        ex_alu_st_mask <= 6'b011000;
                        end
                    3:  begin // ROR
                        ex_alu_OP <= ALU_SR;
                        ex_alu_carry_in <= 2'b10; // C <= P[C]
                        ex_alu_st_mask <= 6'b111000;
                        end
                    4:  begin // copy from X: STX, TXA, TXY
                        ex_sb_src <= REG_X;
                        ex_res_src <= RES_SB; 
                        case(op_b)
                            2:          ex_res_dst <= REG_A;
                            6:          ex_res_dst <= REG_S;
                            default:    ex_res_dst <= REG_D;
                        endcase
                        end
                    5:  begin // copy to X: LDX, TAX, TSX
                        ex_res_dst <= REG_X;
                        ex_res_src <= (op_b == 2 || op_b == 6) ? RES_SB : RES_DB; 
                        ex_sb_src <= (op_b == 6) ? REG_S : REG_A;
                        end
                    6:  begin // DEC
                        // ex_alu_OP <= ALU_ADD;            //already default
                        // ex_alu_carry_in <= 2'b00;        //already default
                        if (op_b == 2)  ex_alu_INVBI <= 1;  // op(reg,-1)
                        else            ex_alu_INVAI <= 1;  // op(-1,MEM)
                        ex_alu_st_mask <= 6'b110000;
                        end
                    7:  begin // INC, NOP
                        // ex_alu_OP <= ALU_ADD;
                        ex_alu_carry_in <= 2'b01; // C=1
                        ex_alu_st_mask <= 6'b110000;
                        if (op_b==2) begin
                            ex_alu_st_mask <= 0;
                            ex_res_dst <= REG_Z;
                        end
                        end
                endcase
            end
        endcase
    end

    integer cycle = 0;
    always @(posedge i_clk ) begin
        if (i_rst) cycle <= 0;
        else if (state==T_BOOT) cycle <= 0;
        else cycle <= cycle+1;
    end

    //state machine
    always @(posedge i_clk ) begin
        if (i_rst) state <= T_BOOT;
        else case(state)
            T0:       state <= T1;
            T1:       state <= initial_state;
            T2_ZPG:   state <= T0;
            T2_ZPGXY: state <= T3_ZPGXY;
            T3_ZPGXY: state <= T0;
            T2_ABS:   state <= T3_ABS;
            T3_ABS:   state <= T0;
            T2_ABSXY: state <= T3_ABSXY;
            T3_ABSXY: state <= state_skip ? T0 : T4_ABSXY;
            T4_ABSXY: state <= T0;
            T2_XIND:  state <= T3_XIND;
            T3_XIND:  state <= T4_XIND;
            T4_XIND:  state <= T5_XIND;
            T5_XIND:  state <= T0;
            T2_INDY:  state <= T3_INDY;
            T3_INDY:  state <= T4_INDY;
            T4_INDY:  state <= state_skip ? T0 : T5_INDY;
            T5_INDY:  state <= T0;
            T_BOOT:     state <= T0;
            default:  state <= T_JAM;
        endcase
    end

    // addr control
    logic [7:0] zeropage = 8'h00;
    always @(*) begin
        addr = pc;
        IPC = 0;

        // default alu ops:
        // result = data + idx_XY
        db_src = DB_DL;
        sb_src = idx_XY ? REG_X : REG_Y;
        res_src = RES_ADD;
        res_dst = REG_Z;

        alu_OP = ALU_ADD;
        alu_INVAI = 0;
        alu_INVBI = 0;
        alu_AORB = 0;
        alu_st_mask = 0;        // do not update status flag
        alu_carry_in = 2'b00;   // carry in = 0

        case(state)
            T0:         begin
                        IPC = 1;                // fetch pc, pc++

                        //execute op
                        db_src = ex_db_src;
                        sb_src = ex_sb_src;
                        alu_OP = ex_alu_OP;
                        alu_INVAI = ex_alu_INVAI;
                        alu_INVBI = ex_alu_INVBI;
                        alu_AORB = ex_alu_AORB;
                        alu_carry_in = ex_alu_carry_in;

                        end

            T1:         begin
                        IPC = !op_SB;           // fetch pc, pc++ unless new opcode is only 1 byte

                        alu_st_mask = ex_alu_st_mask;
                        db_src = ex_db_src;
                        sb_src = ex_sb_src;
                        res_src = ex_res_src;
                        res_dst = ex_res_dst;

                        end

            T2_ZPG:     addr = {zeropage, db};  // fetch {0,BAL}

            T2_ZPGXY:   addr = {radh,radl};     // hold addr, compute LL = BAL + X/Y
            T3_ZPGXY:   addr = {zeropage,res};  // fetch {0,LL}

            T2_ABS:     begin
                        sb_src = REG_Z;         // alu nop: hold LL in add
                        IPC = 1;                // fetch BAH
                        end
            T3_ABS:     addr = {db,res};        // fetch {BAH,BAL}

            T2_ABSXY:   IPC = 1;                // compute LL = BAL + X/Y, fetch BAH
            T3_ABSXY:   begin
                        addr = {db,res};        // fetch {BAH,LL}, assuming no carry
                        sb_src = REG_Z;         // compute HH = BAH+1
                        alu_carry_in = 2'b01;
                        state_skip = !carry_out;// if no carry, skip T4_ABS
                        end
            T4_ABSXY:   addr = {res,radl};      // fetch {HH,LL}

            T2_XIND:    addr = {zeropage, db};  // fetch {0,BAL} (discarded), compute BAL+X
            T3_XIND:    begin
                        addr = {zeropage, res}; // fetch LL = {0,BAL+X}
                        db_src = DB_AL;         // compute BAL+X+1;
                        sb_src = REG_Z;
                        alu_carry_in = 2'b01;
                        end
            T4_XIND:    begin
                        sb_src = REG_Z;         // hold LL in add
                        addr = {zeropage, res}; // fetch {0,BAL+X+1}
                        end
            T5_XIND:    addr = {db, res};       // fetch {HH,LL}

            T2_INDY:    begin
                        addr = {zeropage, db};  // fetch BAL = {00,IAL}
                        sb_src = REG_Z;         // compute IAL + 1
                        alu_carry_in = 2'b01;
                        end
            T3_INDY:    addr = {zeropage, res}; // fetch BAH = {00,IAL+1}, compute BAL + Y
            T4_INDY:    begin
                        addr = {db, res};       // fetch {BAH,BAL+Y}, assuming no carry
                        sb_src = REG_Z;         // compute BAH+1
                        alu_carry_in = 2'b01;
                        state_skip = !carry_out;// if no carry, skip T5
                        end
            T5_INDY:    addr = {res, radl};     // fetch {BAH+1,BAL+Y}
        endcase
    end


endmodule

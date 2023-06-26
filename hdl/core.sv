module core #(
    parameter NMI_VECTOR=16'hfffa,
    parameter RST_VECTOR=16'hfffc,
    parameter IRQ_VECTOR=16'hfffe)
    (
    input  logic i_clk, i_rst,
    input  logic [7:0] i_data,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,

    output logic [15:0] addr,
    output logic [7:0] dor,
    output logic RW,
    output logic sync,
    output logic jam
    );

    // busses
    // db: data bus: typically memory read, but can be addr_lo or zero
    // sb: secondry bus: read from registers (a,x,y,s, zero)
    // res: result bus: update register (a,x,y,s,d [mem write], z[disabled])
    logic [7:0] db, sb, res;

    // registers
    logic [15:0] pc, pcsel, nextpc;
    logic [7:0] a, s, x, y, p;
    logic [7:0] pcl, pch;
    logic [7:0] ir;
    logic [7:0] add;

    // control signals
    logic [2:0] adl_src,adh_src;
    logic ipc;
    logic inc_pc;
    logic inc_db;
    logic dec_db;
    logic inc_sb;
    logic dec_sb;
    logic push, pop, rpop;
    logic skip;
    logic exec;
    logic save;
    logic jump;
    logic holdalu;

    // pc logic
    assign pcsel = jump ? addr : pc;
    assign nextpc = (inc_pc || jump) ? pcsel + 1 : pcsel;

    // interrupt
    logic nmi, irq, cli;
    always @(posedge i_clk ) begin
        if (i_rst) begin
            nmi <= 0;
            irq <= 0;
        end else begin
            nmi <= NMI ? 1 : nmi & ~cli;
            irq <= IRQ ? 1 : irq & ~cli;
        end
    end

    logic [7:0] adl,adh;
    always @(*) begin
        if (push || rpop) begin
            adl = s;
            adh = STACKPAGE;
        end else begin
            case(adl_src)
                ADDR_PC: adl = pc[7:0];
                ADDR_DB: adl = db;
                ADDR_RES: adl = res;
                ADDR_ADD: adl = add;
                ADDR_Z: adl = 8'b0;
                ADDR_INT: adl = irq ? IRQ_VECTOR[7:0]: NMI_VECTOR[7:0];
                default: adl = pcl; //ADDR_HOLD
            endcase
            case(adh_src)
                ADDR_PC: adh = pc[15:8];
                ADDR_DB: adh = db;
                ADDR_RES: adh = res;
                ADDR_ADD: adh = add;
                ADDR_Z: adh = 8'b0;
                ADDR_INT: adh = irq ? IRQ_VECTOR[15:8]: NMI_VECTOR[15:8];
                default: adh = pch; //ADDR_HOLD
            endcase
        end
    end
    assign addr = {adh, adl};


    logic [2:0] res_dst, op_res_dst, ctl_res_dst;
    assign res_dst = push ? REG_DATA : save ? op_res_dst : ctl_res_dst;
    always @(posedge i_clk ) begin
        
        if (i_rst) begin
            a <= 0;
            x <= 0;
            y <= 0;
            s <= 0;
            p <= FL_I;              //disable IRQ on reset
            pch <= 0;
            pcl <= 0;
            pc <= RST_VECTOR;
            rpop <= 0;

        end else begin
            a <= a;
            x <= x;
            y <= y;
            s <= push ? s-1 : pop ? s+1 : s;
            p <= ~clear_mask & (set_mask | (alu_mask & alu_status) | (~alu_mask & p));
            rpop <= pop;

            pcl <= adl;
            pch <= adh;
            pc <= nextpc;

            case(res_dst)
                REG_A: a <= res;
                REG_X: x <= res;
                REG_Y: y <= res;
                REG_S: s <= res;
                REG_P: p <= res & ~FL_BU; //ignore break flags
                default: begin end
            endcase
        end
    end

    assign RW = !(res_dst == REG_DATA);
    assign dor = RW ? 0 : res;

    // bus source select
    logic [2:0] db_src, ctl_db_src, op_db_src;
    logic [2:0] sb_src, ctl_sb_src, op_sb_src;
    logic [1:0] res_src, ctl_res_src, op_res_src;

    always @(*) begin
        db_src = (exec || save) ? op_db_src : (inc_sb || dec_sb) ? DB_Z : ctl_db_src;
        sb_src = (exec || save) ? op_sb_src : (inc_db || dec_db) ? REG_Z : ctl_sb_src;
        res_src = save ? op_res_src : ctl_res_src;
        case(db_src)
            DB_DATA:    db = i_data;
            DB_PCL:     db = pc[7:0];
            DB_PCH:     db = pc[15:8];
            DB_S:       db = s;
            DB_P:       db = p | (!irq && FL_BU); // set break flags unless irq 
            DB_A:       db = a;
            default:    db = 8'h0;
        endcase
        case(sb_src)
            REG_A: sb = a;
            REG_X: sb = x;
            REG_Y: sb = y;
            REG_S: sb = s;
            REG_ADD: sb = add;
            REG_DATA: sb = i_data;
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
    logic [2:0] alu_OP;
    logic [7:0] alu_out;
    logic [1:0] alu_carry_in;
    logic ci, aluN, aluZ, aluC, aluV;
    logic ai_inv, bi_inv;
    logic [7:0] ai, bi;
    assign ai = ai_inv ? ~sb : sb;
    assign bi = bi_inv ? ~db : db;
    assign ci = alu_carry_in==CARRY_C ? p[0] : alu_carry_in==CARRY_P ? 1 : 0;

    always @(*) begin
        if (exec) begin
            alu_OP = op_alu_OP;
            ai_inv = op_ai_inv;
            bi_inv = op_bi_inv;
            alu_carry_in = op_carry_in;
            alu_mask = op_alu_mask;
            set_mask = op_set_mask;
            clear_mask = op_clear_mask;
        end else begin
            alu_OP = ALU_ADD;
            ai_inv = dec_db;
            bi_inv = dec_sb;
            alu_carry_in = (inc_db || inc_sb) ? CARRY_P : CARRY_Z;
            alu_mask = '0;
            set_mask = '0;
            clear_mask = '0;
        end
    end

    alu u_alu(
        .ai  (ai  ),
        .bi  (bi  ),
        .ci  (ci  ),
        .op  (alu_OP),
        .out (alu_out),
        .N   (aluN),
        .V   (aluV),
        .Z   (aluZ),
        .C   (aluC)
    );

    // register alu
    logic [7:0] alu_status, ralu_status, alu_mask;
    logic[7:0] set_mask, clear_mask;
    assign alu_status = {aluN, aluV, 4'b0, aluZ, aluC};
    always @(posedge i_clk ) begin
        if (i_rst) begin
            add <= 8'b0;
            ralu_status <= 8'h0;
        end else if(holdalu) begin
            add <= add;
            ralu_status <= ralu_status;
        end else begin
            add <= alu_out;
            ralu_status <= alu_status;
        end
    end


    // state
    logic [5:0] state, initial_state;

    // ir fetch
    // assign sync = state == T0_FETCH;
    logic [7:0] opcode;
    always @(posedge i_clk ) begin
        if (i_rst)          ir <= 8'h4c; //jmp (from RESET_VECTOR)
        else if (state==T1_DECODE) ir <= i_data;
    end
    assign opcode = (state==T1_DECODE) ? i_data : ir;

    logic single_byte;
    logic idx_XY;           // select index X(1) or Y(0) for abs,X/Y and zpg,X/Y ops
    logic rd_op, ld_op, st_op, rmw_op;

    logic [2:0] op_alu_OP;
    logic op_ai_inv;   // alu ai src
    logic op_bi_inv;   // alu bi src
    logic [1:0] op_carry_in; // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]
    logic [7:0] op_alu_mask, op_set_mask, op_clear_mask;  // masks to update status

    decode u_decode(
        .i_clk         (i_clk         ),
        .i_rst         (i_rst         ),
        .opcode        (opcode        ),
        .pstatus       (p             ),
        .initial_state (initial_state ),
        .single_byte   (single_byte   ),
        .idx_XY        (idx_XY   ),
        .read          (rd_op          ),
        .load          (ld_op          ),
        .store         (st_op         ),
        .rmw           (rmw_op           ),
        .sb_src        (op_sb_src        ),
        .db_src        (op_db_src        ),
        .res_src       (op_res_src       ),
        .res_dst       (op_res_dst       ),
        .alu_OP        (op_alu_OP        ),
        .ai_inv        (op_ai_inv        ),
        .bi_inv        (op_bi_inv        ),
        .carry_in      (op_carry_in      ),
        .alu_mask      (op_alu_mask      ),
        .set_mask      (op_set_mask      ),
        .clear_mask    (op_clear_mask    )
    );

    //state machine
    logic jump_ind;
    always @(posedge i_clk ) begin
        jump_ind <= 0;
        if (i_rst) begin
            state <= T_BOOT;
        end
        else case(state)
            T0_FETCH:       state <= T1_DECODE;
            T1_DECODE:       state <= initial_state;
            T2_ZPG:   state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_ZPGXY: state <= T3_ZPGXY;
            T3_ZPGXY: state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_ABS:   state <= T3_ABS;
            T3_ABS:   state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_ABSXY: state <= T3_ABSXY;
            T3_ABSXY: state <= !skip ? T4_ABSXY : rmw_op ? T_RMW_EXEC : T0_FETCH;
            T4_ABSXY: state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_XIND:  state <= T3_XIND;
            T3_XIND:  state <= T4_XIND;
            T4_XIND:  state <= T5_XIND;
            T5_XIND:  state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_INDY:  state <= T3_INDY;
            T3_INDY:  state <= T4_INDY;
            T4_INDY:  state <= !skip ? T5_INDY : rmw_op ? T_RMW_EXEC : T0_FETCH;
            T5_INDY:  state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T_RMW_EXEC:  state <= T_RMW_STORE;
            T_RMW_STORE: state <= T0_FETCH;

            T2_JUMP:   begin
                        // register whether we are on 1st step of indirect jump
                        jump_ind <= (ir[2] & ir[5]) & !jump_ind;
                        state <= T3_JUMP;
                      end 
            // fetching new address
            // if abs jump, this is new opcode, go to decode
            // if ind jump, this is new pc, redo jump
            T3_JUMP:   begin
                        state <= jump_ind ? T2_JUMP : T1_DECODE;
                        jump_ind <= jump_ind;
                      end

            T2_BRANCH:   state <= T3_BRANCH;
            T3_BRANCH:   state <= skip ? T1_DECODE : T4_BRANCH;
            T4_BRANCH:   state <= T1_DECODE;

            T2_PUSH:     state <= T0_FETCH;
            T2_POP:      state <= T3_POP;
            T3_POP:      state <= T0_FETCH;
            
            T2_BRK:      state <= T3_BRK;
            T3_BRK:      state <= T4_BRK;
            T4_BRK:      state <= T5_BRK;
            T5_BRK:      state <= T2_JUMP;

            T2_RTI:      state <= T3_RTI;
            T3_RTI:      state <= T4_RTI;
            T4_RTI:      state <= T5_RTI;
            T5_RTI:      state <= T3_JUMP;

            T2_RTS:      state <= T3_RTS;
            T3_RTS:      state <= T4_RTS;
            T4_RTS:      state <= T5_RTS;
            T5_RTS:      state <= T0_FETCH;

            T2_JSR:      state <= T3_JSR;
            T3_JSR:      state <= T4_JSR;
            T4_JSR:      state <= T5_JSR;
            T5_JSR:      state <= T3_JUMP;

            T_BOOT:   state <= T2_JUMP;
            default:  state <= T_JAM;

        endcase
    end

    // control
    always @(*) begin
        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 0;
        ctl_db_src  = DB_DATA;
        ctl_sb_src  = idx_XY ? REG_X : REG_Y;
        ctl_res_src = RES_ADD;
        ctl_res_dst = REG_Z;
        inc_db      = 0;
        dec_db      = 0;
        inc_sb      = 0;
        dec_sb      = 0;
        holdalu     = 0;
        push        = 0;
        pop         = 0;
        skip        = 0;
        exec        = 0;
        save        = 0;
        jump        = 0;
        jam         = 0;
        cli         = 0;
        sync        = 0;

        case(state)
            T0_FETCH:   begin
                        sync = 1;               // signal new instruction
                        inc_pc = 1;             // increment pc
                        exec = rd_op;           // on read, execute instruction
                        save = ld_op;           // on load, save result
                        end

            T1_DECODE:  begin
                        inc_pc = !single_byte;        // increment pc unless this is single byte opcode
                        save = rd_op;           // on read, save result
                        end

            T2_ZPG:     begin
                        adl_src = ADDR_DB;      // fetch {0,BAL}
                        adh_src = ADDR_Z;
                        save = st_op;           // on store, save result
                        end

            T2_ZPGXY:   begin
                        adl_src = ADDR_HOLD;    // hold addr while we compute LL = BAL + X/Y
                        adh_src = ADDR_HOLD;
                        end

            T3_ZPGXY:   begin
                        adl_src = ADDR_RES;     // fetch {0,LL}
                        adh_src = ADDR_Z;
                        save = st_op;           // on store, save result
                        end

            T2_JUMP,
            T2_ABS:     begin
                        ctl_sb_src = REG_Z;         // alu nop: hold LL in add
                        inc_pc = 1;             // next pc = pc++
                        end

            T3_ABS:     begin
                        adl_src = ADDR_ADD;      // fetch {BAH,BAL}
                        adh_src = ADDR_DB;
                        save = st_op;           // on store, save result
                        end

            T2_ABSXY:   begin
                        // compute LL = BAL + X/Y
                        inc_pc = 1;                // fetch BAH, pc++
                        end                

            T3_ABSXY,
            T4_INDY:    begin
                        adl_src = ADDR_RES;      // fetch {BAH,LL}, assuming no carry
                        adh_src = ADDR_DB;
                        if (ralu_status[0]) begin
                            // low address overflowed, need to increment ADH
                            inc_db = 1;
                        end else begin
                            skip = 1;
                            save = st_op;
                        end
                        end

            T4_ABSXY,
            T5_INDY:    begin
                        adl_src = ADDR_HOLD;      // fetch {HH,LL} with incremented HH
                        adh_src = ADDR_RES;
                        save = st_op;
                        end

            T2_XIND:    begin
                        adl_src = ADDR_DB;      // fetch {0,BAL}
                        adh_src = ADDR_Z;
                        end                
            T3_XIND:    begin
                        adl_src = ADDR_RES;     // fetch LL = {0,BAL+X}
                        adh_src = ADDR_Z;
                        ctl_sb_src = REG_ADD;
                        inc_sb = 1;             // increment result = BAL+X+1
                        end
            T4_XIND:    begin
                        ctl_sb_src = REG_Z;         // hold LL in add
                        adl_src = ADDR_RES;     // fetch {0,BAL+X+1}
                        adh_src = ADDR_Z;
                        end
            T5_XIND:    begin
                        adl_src = ADDR_RES;      // fetch {BAH,LL}, assuming no carry
                        adh_src = ADDR_DB;
                        save = st_op;
                        end

            T2_INDY:    begin
                        adl_src = ADDR_DB;      // fetch {0,IAL}
                        adh_src = ADDR_Z;
                        inc_db = 1;             // compute IAL + 1
                        end

            T3_INDY:    begin
                        adl_src = ADDR_RES;     // fetch BAH = {00,IAL+1}, compute BAL + Y
                        adh_src = ADDR_Z;
                        end

            // T4_INDY-T5_INDY handled with T3_ABSXY-T4_ABSXY

            T_RMW_EXEC: begin
                        adl_src = ADDR_HOLD;
                        adh_src = ADDR_HOLD;
                        exec = 1;
                        end

            T_RMW_STORE:begin
                        adl_src = ADDR_HOLD;
                        adh_src = ADDR_HOLD;
                        save = 1;
                        end

            T2_BRANCH:  begin
                        inc_pc = 1;             // next pc = pc++
                        ctl_db_src = DB_PCL;        // calc branch offset      
                        ctl_sb_src = REG_DATA;
                        end

            T3_BRANCH:  begin
                        adl_src = ADDR_ADD;
                        adh_src = ADDR_HOLD;
                        jump = 1;              // addr -> pc
                        if (ralu_status[0] && !ralu_status[7]) begin
                             // C=1, N=0: jumped to higher page, pch must be incremented
                            ctl_db_src = DB_PCH;
                            inc_db = 1;
                        end else if (ralu_status[6] && ralu_status[7]) begin
                             // V=1, N=1: jumped to lower page, pch must be decremented
                            ctl_db_src = DB_PCH;
                            dec_db = 1;
                        end else begin
                            // pch is valid, this cycle is our T0_FETCH fetch
                            inc_pc = 1;         
                            sync = 1;
                            skip = 1;
                        end
                        end

            T4_BRANCH: begin
                        adl_src = ADDR_ADD;
                        adh_src = ADDR_HOLD;
                        jump = 1;              // addr -> pc
                        inc_pc = 1;            // nextpc = pc++
                        sync = 1;
                        end

            T5_RTS,
            T3_JUMP:     begin
                        adl_src = ADDR_ADD;     // fetch new pc
                        adh_src = ADDR_DB;
                        jump = 1;               // update pc
                        sync = !jump_ind;       //
                        end

            T2_PUSH:   begin
                            ctl_db_src = op_db_src;
                            ctl_res_src = RES_DB;
                            ctl_res_dst = REG_DATA;
                            push = 1;
                        end

            T2_POP:     pop = 1;
            T3_POP:     begin end //data is fetched...

            T2_BRK:     begin
                        //push pch
                        ctl_db_src = DB_PCH;
                        ctl_res_src = RES_DB;
                        ctl_res_dst = REG_DATA;
                        push = 1;
                        end

            T3_BRK:    begin
                        //push pcl
                        ctl_db_src = DB_PCL;
                        ctl_res_src = RES_DB;
                        ctl_res_dst = REG_DATA;
                        push = 1;
                        end

            T4_BRK:    begin
                        //push p
                        ctl_db_src = DB_P;
                        ctl_res_src = RES_DB;
                        ctl_res_dst = REG_DATA;
                        push = 1;
                        cli = 1;
                        end

            T5_BRK:    begin
                        adl_src = ADDR_INT;
                        adh_src = ADDR_INT;
                        jump = 1;
                        inc_pc = 1;
                        end


            T2_RTS,T3_RTS,          //RTS: T2: pop pcl, T3: fetch pcl, pop pch
            T2_RTI,                 //RTI: T2 pop p
            T3_RTI:     pop = 1;    //fetch p, pop pcl

            T4_RTI:     begin       //store p, fetch pcl, pop pch
                            ctl_db_src = DB_DATA;
                            ctl_res_src = RES_DB;
                            ctl_res_dst = REG_P;
                            pop = 1;
                        end
            T4_RTS,
            T5_RTI:     ctl_sb_src = REG_Z; // store pcl in alu, fetch pch



            T2_JSR:     ctl_sb_src = REG_Z;      // alu nop: hold ADL in alu
            T3_JSR:     begin
                        holdalu = 1; // continue to hold ADL in alu
                        //push pch
                        ctl_db_src = DB_PCH;
                        ctl_res_src = RES_DB;
                        ctl_res_dst = REG_DATA;
                        push = 1;
                        end

            T4_JSR:    begin
                        holdalu = 1; // continue to hold ADL in alu
                        //push pcl
                        ctl_db_src = DB_PCL;
                        ctl_res_src = RES_DB;
                        ctl_res_dst = REG_DATA;
                        push = 1;
                        end
            T5_JSR:    begin
                        holdalu = 1; // continue to hold ADL in alu
                        // load ADH
                        end

            T_BOOT:     inc_pc = 1; // fetch low reset addr


            default:    jam = 1;
        endcase
    end

endmodule

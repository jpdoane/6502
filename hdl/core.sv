`include "defs.svi"


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
    output logic SYNC,
    output logic JAM
    );

    // busses
    // db: data bus: typically memory read, but can be addr_lo or zero
    // sb: secondry bus: read from registers (a,x,y,s, zero)
    // res: result bus: update register (a,x,y,s,d [mem write], z[disabled])
    logic [7:0] db, sb, res;

    // registers
    logic [15:0] pc, pcsel, nextpc;

    // pc logic
    logic jump, pc_inc;
    assign pcsel = jump ? addr : pc;
    assign nextpc = (pc_inc || jump) ? pcsel + 1 : pcsel;

    logic [7:0] a, s, x, y, p;
    logic [7:0] radl, radh;
    logic [7:0] ir;
    logic [7:0] add;
    logic IPC;

    logic [2:0] res_dst;

    always @(posedge i_clk ) begin
        if (i_rst) begin
            a <= 0;
            x <= 0;
            y <= 0;
            s <= 0;
            radh <= 0;
            radl <= 0;
            pc <= RST_VECTOR;
            //p updated elsewhere...
        end else begin
            a <= a;
            x <= x;
            y <= y;
            s <= s;

            radh <= addr[15:8];
            radl <= addr[7:0];
            pc <= nextpc;

            case(res_dst)
                REG_A: a <= res;
                REG_X: x <= res;
                REG_Y: y <= res;
                REG_S: s <= res;
                REG_P: p <= res & 8'hef; // clear break flag on p read
                default: begin end
            endcase
        end
    end

    assign RW = !(res_dst == REG_DATA);
    assign dor = RW ? 0 : res;

    // bus source select
    logic [2:0] db_src;
    logic [2:0] sb_src;
    logic [1:0] res_src;
    always @(*) begin
        case(db_src)
            DB_DATA:    db = i_data;
            DB_PCL:     db = pc[7:0];
            DB_PCH:     db = pc[15:8];
            DB_S:       db = s;
            DB_P:       db = p;
            DB_A:       db = a;
            default:    db = 8'h0;
        endcase
        case(sb_src)
            REG_A: sb = a;
            REG_X: sb = x;
            REG_Y: sb = y;
            REG_S: sb = s;
            REG_P: sb = p | 8'h10; // set break flag on p write
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

    // alu source select
    logic [1:0] ai_src, bi_src;
    logic [7:0] ai, bi;
    always @(*) begin
        case(ai_src)
            ADD_BUS:    ai = sb;
            ADD_INVBUS: ai = ~sb;
            ADD_N:      ai = 8'hff;
            default:    ai = 8'h0;
        endcase
        case(bi_src)
            ADD_BUS:    bi = db;
            ADD_INVBUS: bi = ~db;
            ADD_N:      bi = 8'hff;
            default:    bi = 8'h0;
        endcase
    end

    //alu
    logic [2:0] alu_OP;
    logic [7:0] alu_out;
    logic [1:0] alu_carry_in; // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]
    logic ci, aluN, aluZ, aluC, aluV;

    assign ci = alu_carry_in[1] ? p[0] : alu_carry_in[0];
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

    // register alu and p
    logic [7:0] alu_status, ralu_status, alu_mask;
    logic[7:0] set_mask, clear_mask;
    assign alu_status = {aluN, aluV, 4'b0, aluZ, aluC};
    always @(posedge i_clk ) begin
        if (i_rst) begin
            add <= 8'b0;
            p <= 8'h4;              //set IRQ disable on reset
            ralu_status <= 8'h0;
        end else begin
            add <= alu_out;
            ralu_status <= alu_status;
            p <= ~clear_mask & (set_mask | (alu_mask & alu_status) | (~alu_mask & p));
        end
    end


    // state
    logic [4:0] state, initial_state;
    logic state_skip;

    // ir fetch
    // assign SYNC = state == T0_FETCH;
    logic [7:0] opcode;
    always @(posedge i_clk ) begin
        if (i_rst)          ir <= 8'h4c; //jmp (from RESET_VECTOR)
        else if (state==T1_DECODE) ir <= i_data;
    end
    assign opcode = (state==T1_DECODE) ? i_data : ir;

    logic op_SB;
    logic idx_XY;           // select index X(1) or Y(0) for abs,X/Y and zpg,X/Y ops
    logic rd_op, ld_op, st_op, rmw_op;

    logic [2:0] ex_sb_src;   // sb bus source (registers)
    logic [2:0] ex_db_src;   // db bus source (data/addr)
    logic [1:0] ex_res_src;  // source of result (alu, sb, db)
    logic [2:0] ex_res_dst;  // location to store result (registers, mem)
    logic [2:0] ex_alu_OP;
    logic [1:0] ex_ai_src;   // alu ai src
    logic [1:0] ex_bi_src;   // alu bi src
    logic [1:0] ex_carry_in; // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]
    logic [7:0] ex_alu_mask, ex_set_mask, ex_clear_mask;  // masks to update status

    decode u_decode(
        .i_clk         (i_clk         ),
        .i_rst         (i_rst         ),
        .opcode        (opcode        ),
        .pstatus       (p             ),
        .initial_state (initial_state ),
        .single_byte   (op_SB   ),
        .idx_XY        (idx_XY   ),
        .read          (rd_op          ),
        .load          (ld_op          ),
        .store         (st_op         ),
        .rmw           (rmw_op           ),
        .sb_src        (ex_sb_src        ),
        .db_src        (ex_db_src        ),
        .res_src       (ex_res_src       ),
        .res_dst       (ex_res_dst       ),
        .alu_OP        (ex_alu_OP        ),
        .ai_src        (ex_ai_src        ),
        .bi_src        (ex_bi_src        ),
        .carry_in      (ex_carry_in      ),
        .alu_mask      (ex_alu_mask      ),
        .set_mask      (ex_set_mask      ),
        .clear_mask    (ex_clear_mask    )
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
            T3_ABSXY: state <= !state_skip ? T4_ABSXY : rmw_op ? T_RMW_EXEC : T0_FETCH;
            T4_ABSXY: state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_XIND:  state <= T3_XIND;
            T3_XIND:  state <= T4_XIND;
            T4_XIND:  state <= T5_XIND;
            T5_XIND:  state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_INDY:  state <= T3_INDY;
            T3_INDY:  state <= T4_INDY;
            T4_INDY:  state <= !state_skip ? T5_INDY : rmw_op ? T_RMW_EXEC : T0_FETCH;
            T5_INDY:  state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T_RMW_EXEC:  state <= T_RMW_STORE;
            T_RMW_STORE: state <= T0_FETCH;

            T2_JMP:   begin
                        // register whether we are on 1st step of indirect jump
                        jump_ind <= ir[5] & !jump_ind;
                        state <= T3_JMP;
                      end 
            // fetching new address
            // if abs jump, this is new opcode, go to decode
            // if ind jump, this is new pc, redo jump
            T3_JMP:   begin
                        state <= jump_ind ? T2_JMP : T1_DECODE;
                        jump_ind <= jump_ind;
                      end

            T2_BRANCH:   state <= T3_BRANCH;
            T3_BRANCH:   state <= state_skip ? T1_DECODE : T4_BRANCH;
            T4_BRANCH:   state <= T1_DECODE;

            T2_STACK:    state <= T3_STACK;
            T3_STACK:    state <= st_op ? T1_DECODE : T0_FETCH;
            
            T_BOOT:   state <= T2_JMP;
            default:  state <= T_JAM;
        endcase
    end

    // addr control
    logic [7:0] zeropage = 8'h00;
    logic [7:0] stackpage = 8'h01;
    logic compute_result, save_result;

    always @(*) begin
        addr = pc;

        set_mask = 0;
        clear_mask = 0;
        state_skip = 0;
        compute_result = 0;
        save_result = 0;
        jump = 0;
    
        // default alu when not processing result:
        // result = data + idx_XY
        // cin = 0, st_mask = 0, do not store result
        ai_src = ADD_BUS;
        bi_src = ADD_BUS;
        db_src = DB_DATA;
        sb_src = idx_XY ? REG_X : REG_Y;
        alu_OP = ALU_ADD;
        alu_carry_in = 0;
        alu_mask = 0;
        res_src = RES_ADD;
        res_dst = REG_Z;

        jump = 0;
        pc_inc = 0;
        SYNC = 0;
        JAM = 0;
        
        case(state)
            T0_FETCH:         begin
                        SYNC = 1;
                        pc_inc = 1;             // next pc = pc++
                        compute_result = rd_op; // compute read op
                        save_result = ld_op;    // save load op
                        end

            T1_DECODE:  begin
                        // fetch pc, next pc = pc++ unless new opcode is only 1 byte
                        pc_inc = !op_SB;
                        save_result = rd_op;    // store result of read op
                        end

            T2_ZPG:     begin
                        addr = {zeropage, db};  // fetch {0,BAL}
                        save_result = st_op;
                        end

            T2_ZPGXY:   addr = {radh,radl};     // hold addr, compute LL = BAL + X/Y
            T3_ZPGXY:   begin
                        addr = {zeropage,res};  // fetch {0,LL}
                        save_result = st_op;
                        end

            T2_JMP,
            T2_ABS:     begin
                        sb_src = REG_Z;         // alu nop: hold LL in add
                        // fetch BAH
                        pc_inc = 1;             // next pc = pc++
                        end
            T3_ABS:     begin
                        addr = {db,res};        // fetch {BAH,BAL}
                        save_result = st_op;
                        end

            T2_ABSXY:   begin
                        // compute LL = BAL + X/Y
                        //fetch BAH
                        pc_inc = 1;                // fetch pc, pc++
                        end                
            T3_ABSXY:   begin
                        addr = {db,res};        // fetch {BAH,LL}, assuming no carry
                        if (ralu_status[0]) begin //carry
                            sb_src = REG_Z;       // inc BAH
                            alu_carry_in = 2'b01;
                        end else begin
                            state_skip = 1;
                            save_result = st_op;
                        end
                        end
            T4_ABSXY:   begin
                        addr = {res,radl};      // fetch {HH,LL}
                        save_result = st_op;
                        end

            T2_XIND:    addr = {zeropage, db};  // fetch {0,BAL} (discarded), compute BAL+X
            T3_XIND:    begin
                        addr = {zeropage, res}; // fetch LL = {0,BAL+X}
                        db_src = DB_Z;         // compute BAL+X+1;
                        sb_src = REG_ADD;
                        alu_carry_in = 2'b01;
                        end
            T4_XIND:    begin
                        sb_src = REG_Z;         // hold LL in add
                        addr = {zeropage, res}; // fetch {0,BAL+X+1}
                        end
            T5_XIND:    begin
                        addr = {db, res};       // fetch {HH,LL}
                        save_result = st_op;
                        end

            T2_INDY:    begin
                        addr = {zeropage, db};  // fetch BAL = {00,IAL}
                        sb_src = REG_Z;         // compute IAL + 1
                        alu_carry_in = 2'b01;
                        end
            T3_INDY:    addr = {zeropage, res}; // fetch BAH = {00,IAL+1}, compute BAL + Y
            T4_INDY:    begin
                        addr = {db, res};       // fetch {BAH,BAL+Y}, assuming no carry
                        if (ralu_status[0]) begin
                            sb_src = REG_Z;         // inc BAH
                            alu_carry_in = 2'b01;
                        end else begin
                            state_skip = 1;
                            save_result = st_op;
                        end
                        end
            T5_INDY:    begin
                        addr = {res, radl};     // fetch {BAH+1,BAL+Y}
                        save_result = st_op;
                        end
            T_RMW_EXEC: begin
                        addr = {radh,radl};     // hold addr
                        compute_result = 1;
                        end
            T_RMW_STORE:begin
                        addr = {radh,radl};     // hold addr
                        save_result = 1;
                        end

            T2_BRANCH:   begin
                        pc_inc = 1;             // next pc = pc++
                        // pcl = offset + pcl
                        db_src = DB_PCL;        
                        sb_src = REG_DATA;
                        end

            T3_BRANCH:  begin
                        addr = {radh,add};     // {pch, offset + pcl}
                        jump = 1;              // addr -> pc
                        if (ralu_status[0] && !ralu_status[7]) begin
                             // C=1, N=0: jumped to higher page, pch must be incremented
                            db_src = DB_PCH;         // inc PCH
                            ai_src = ADD_Z;
                            alu_carry_in = 2'b01;
                        end else if (ralu_status[6] && ralu_status[7]) begin
                             // V=1, N=1: jumped to lower page, pch must be decremented
                            db_src = DB_PCH;         // dec PCH
                            ai_src = ADD_N;
                        end else begin
                            // pch is valid, this will be valid T0_FETCH fetch
                            // nextpc = pc++
                            pc_inc = 1;         
                            SYNC = 1;
                            state_skip = 1;
                        end
                        end

            T4_BRANCH: begin
                        addr = {add,radl};     // {pch, offset + pcl}
                        jump = 1;              // addr -> pc
                        pc_inc = 1;            // nextpc = pc++
                        SYNC = 1;
                        end

            T3_JMP:     begin
                        addr = {db,add};        // fetch new pc
                        jump = 1;               // update pc
                        SYNC = !jump_ind;       //
                        end


            // stack data movement is on db (ptr arithmetic on sb)
            // to push:
            // ex_db_src: what to push
            // ex_res_src: RES_DB
            // ex_res_dst: REG_DATA
            // to pull:
            // ex_db_src: DB_DATA
            // ex_res_src: RES_DB
            // ex_res_dst: what to pull
            T2_STACK:    begin
                        addr = {stackpage,s};
                        if (st_op) begin
                            //push to stack
                            db_src = ex_db_src;
                            res_src = RES_DB;
                            res_dst = REG_DATA;
                        end
                        // inc/dec s
                        sb_src = REG_S;
                        bi_src = st_op ? ADD_N : ADD_Z;    // push: s--
                        alu_carry_in = ld_op ? 'b01 : 0;   // pull: s++
                        end

            T3_STACK:   begin
                        // update stack register
                        res_src = REG_ADD;
                        res_dst = REG_S;
                        
                        // for push, this is T0 and we need to fetch next addr
                        if (st_op) begin
                            pc_inc = 1;             // next pc = pc++
                            SYNC = 1;
                        end
                        

                        // for pull, (s++) loaded to ex_res_dst during next T0
                        if(ld_op) addr = {stackpage,add};
                        end

            T_BOOT:     pc_inc = 1; // fetch low reset addr


            default:    JAM = 1;
        endcase

        // compute and save result
        if (compute_result || save_result) begin
            db_src = ex_db_src;
            sb_src = ex_sb_src;
        end
        if (compute_result) begin
            alu_OP = ex_alu_OP;
            ai_src = ex_ai_src;
            bi_src = ex_bi_src;
            alu_carry_in = ex_carry_in;
            alu_mask = ex_alu_mask;
            set_mask = ex_set_mask;
            clear_mask = ex_clear_mask;
        end
        if (save_result) begin
            res_src = ex_res_src;
            res_dst = ex_res_dst;
        end

    end


endmodule

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


    `include "debug/debug.sv"

    // busses
    // db: data bus: typically memory read, but can be addr_lo or zero
    // sb: secondry bus: read from registers (a,x,y,s, zero)
    logic [7:0] db, sb;

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
    logic push, pop, rpop;
    logic skip;
    logic exec;
    logic save;
    logic jump;
    logic holdalu;
    logic db_write;
    logic sb_a;
    logic sb_x;
    logic sb_y;
    logic db_p;

    // interrupt
    logic nmi, irq, cli;
    assign cli=0;
    always @(posedge i_clk ) begin
        if (i_rst) begin
            nmi <= 0;
            irq <= 0;
        end else begin
            nmi <= NMI ? 1 : cli ? 0 : nmi;
            irq <= IRQ ? !p[2] : cli ? 0 : irq;
        end
    end

    // pc logic
    assign pcsel = jump ? addr : pc;
    assign nextpc = (inc_pc || jump) ? pcsel + 1 : pcsel;


    logic [7:0] adl,adh;
    always @(*) begin
        case(adl_src)
            ADDR_PC: adl = pc[7:0];
            ADDR_DATA: adl = i_data;
            ADDR_ADD: adl = add;
            ADDR_Z: adl = 8'b0;
            ADDR_INT: adl = irq ? IRQ_VECTOR[7:0]: NMI_VECTOR[7:0];
            ADDR_STACK: adl = s;
            default: adl = pcl; //ADDR_HOLD
        endcase
        case(adh_src)
            ADDR_PC: adh = pc[15:8];
            ADDR_DATA: adh = i_data;
            ADDR_ADD: adh = add;
            ADDR_Z: adh = 8'b0;
            ADDR_INT: adh = irq ? IRQ_VECTOR[15:8]: NMI_VECTOR[15:8];
            ADDR_STACK: adh = STACKPAGE;
            default: adh = pch; //ADDR_HOLD
        endcase
    end
    assign addr = {adh, adl};

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
            s <= push ? s-1 : pop ? s+1 : s;
            rpop <= pop;

            pcl <= adl;
            pch <= adh;
            pc <= nextpc;

            a <= sb_a ? sb : a;
            x <= sb_x ? sb : x;
            y <= sb_y ? sb : y;
            p <= db_p ? (db & ~FL_BU) : ~clear_mask & (set_mask | (alu_mask & alu_status) | (~alu_mask & p));

            if (sb_a || sb_x || sb_y || exec ) begin
                p[1] <= ~|sb;   //set Z flag
                p[7] <= sb[7];  //set N flag
            end

        end
    end

    assign RW = !db_write;
    assign dor = db;

    // bus source select
    logic [2:0] db_src,sb_src,dst;
    always @(*) begin
        case(sb_src)
            REG_A: sb = a;
            REG_X: sb = x;
            REG_Y: sb = y;
            REG_S: sb = s;
            REG_ADD: sb = add;
            REG_DATA: sb = i_data;
            default: sb = 8'h0;
        endcase
        case(db_src)
            DB_DATA:    db = i_data;
            DB_PCL:     db = pc[7:0];
            DB_PCH:     db = pc[15:8];
            DB_S:       db = s;
            DB_P:       db = p | (!irq && FL_BU); // set break flags unless irq 
            DB_A:       db = a;
            DB_SB:      db = sb;
            default:    db = 8'h0;
        endcase
    end

    //alu
    logic [2:0] alu_OP;
    logic [7:0] alu_out;
    logic ci, aluN, aluZ, aluC, aluV;
    logic adl_bi;
    logic ai_inv, bi_inv;
    logic [7:0] ai, bi;
    assign ai = ai_inv ? ~sb : sb;
    assign bi = bi_inv ? ~db : adl_bi ? adl : db;

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

    // register alu and branch page flags
    logic [7:0] alu_status, alu_mask;
    logic[7:0] set_mask, clear_mask;
    assign alu_status = {aluN, aluV, 4'b0, aluZ, aluC};
    logic ipage_up, bpage_up, bpage_down;
    always @(posedge i_clk ) begin
        if (i_rst) begin
            add <= 8'b0;
            ipage_up <= 0;
            bpage_up <= 0;
            bpage_down <= 0;
        end else begin 
            add <= holdalu ? add : alu_out;

            // index math is unsigned, so we cross a page boundary if carry bit is set
            ipage_up <= aluC;

            // branch math: ai is the  signed offset, bi is the unsigned base addr
            bpage_up <= bi[7] & !ai[7] & !alu_out[7]; // crossed to next page if addr>127, offset>0, and result <= 127
            bpage_down <= !bi[7] & ai[7] & alu_out[7]; // crossed to prev page if addr<=127, offset<0, and result > 127
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
    logic mem_read, mem_write, reg_write;
    logic alu_en;
    logic [2:0] op_alu_OP;
    logic op_ai_inv;   // alu ai src
    logic op_bi_inv;   // alu bi src
    logic [1:0] op_carry_in; // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]
    logic [7:0] op_alu_mask;  // masks to update status

    decode u_decode(
        .i_clk         (i_clk         ),
        .i_rst         (i_rst         ),
        .opcode        (opcode        ),
        .pstatus       (p             ),
        .initial_state (initial_state ),
        .single_byte   (single_byte   ),
        .idx_XY        (idx_XY   ),
        .mem_read       (mem_read),
        .mem_write      (mem_write),
        .reg_write      (reg_write),
        .sb_src        (op_sb_src        ),
        .db_src        (op_db_src        ),
        .dst       (op_dst       ),
        .alu_en        (alu_en        ),
        .alu_OP        (op_alu_OP        ),
        .ai_inv        (op_ai_inv        ),
        .bi_inv        (op_bi_inv        ),
        .carry_in      (op_carry_in      ),
        .alu_mask      (op_alu_mask      ),
        .set_mask      (set_mask      ),
        .clear_mask    (clear_mask    )
    );
    logic store, load, rmw;
    assign store = !alu_en && mem_write;       //ALU NOP, save to mem (includes push)
    assign load = !alu_en && reg_write;        //ALU NOP, save to reg (includes pull, transfer)
    assign rmw = mem_read && mem_write;        //ALU OP, read/save to/from mem

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
            T2_ZPG:   state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_ZPGXY: state <= T3_ZPGXY;
            T3_ZPGXY: state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_ABS:   state <= T3_ABS;
            T3_ABS:   state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_ABSXY: state <= T3_ABSXY;
            T3_ABSXY: state <= !skip ? T4_ABSXY : rmw ? T_RMW_EXEC : T0_FETCH;
            T4_ABSXY: state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_XIND:  state <= T3_XIND;
            T3_XIND:  state <= T4_XIND;
            T4_XIND:  state <= T5_XIND;
            T5_XIND:  state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_INDY:  state <= T3_INDY;
            T3_INDY:  state <= T4_INDY;
            T4_INDY:  state <= !skip ? T5_INDY : rmw ? T_RMW_EXEC : T0_FETCH;
            T5_INDY:  state <= rmw ? T_RMW_EXEC : T0_FETCH;
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
    logic [2:0] op_db_src,op_sb_src,op_dst;
    always @(*) begin
        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 0;
        holdalu     = 0;
        push        = 0;
        pop         = 0;
        skip        = 0;
        exec        = 0;
        save        = 0;
        jump        = 0;
        jam         = 0;
        sync        = 0;

        //default bus config
        db_src  = DB_DATA;
        sb_src  = idx_XY ? REG_X : REG_Y;

        //register write control 
        db_write=0;      // write db to memory
        sb_a=0;         // save sb -> reg A
        sb_x=0;         // save sb -> reg X
        sb_y=0;         // save sb -> reg Y
        db_p=0;         // save db -> p
        adl_bi=0;

        // default ALU config
        alu_OP = ALU_ADD;
        ai_inv = 0;
        bi_inv = 0;
        ci = 0;
        alu_mask = '0;

        case(state)
            T0_FETCH:   begin
                        sync = 1;               // signal new instruction
                        inc_pc = 1;             // increment pc
                        exec = alu_en && !rmw;  // execute alu ops (except rmw)
                        save = load;            // save non-alu data to reg (includes loads, transfers, pulls) 
                        end

            T1_DECODE:  begin
                        inc_pc = !single_byte;  // increment pc unless this is single byte opcode
                        save = alu_en && !rmw;  // save alu result
                        end

            T2_ZPG:     begin
                        adl_src = ADDR_DATA;      // fetch {0,BAL}
                        adh_src = ADDR_Z;
                        save = store;           // on store, save result
                        end

            T2_ZPGXY:   begin
                        adl_src = ADDR_HOLD;    // hold addr while we compute LL = BAL + X/Y
                        adh_src = ADDR_HOLD;
                        end

            T3_ZPGXY:   begin
                        adl_src = ADDR_ADD;     // fetch {0,LL}
                        adh_src = ADDR_Z;
                        save = store;           // on store, save result
                        end

            T2_JUMP,
            T2_ABS:     begin
                        sb_src = REG_Z;         // alu nop: hold LL in add
                        inc_pc = 1;             // next pc = pc++
                        end

            T3_ABS:     begin
                        adl_src = ADDR_ADD;      // fetch {BAH,BAL}
                        adh_src = ADDR_DATA;
                        save = store;           // on store, save result
                        end

            T2_ABSXY:   begin
                        // compute LL = BAL + X/Y
                        inc_pc = 1;                // fetch BAH, pc++
                        end                

            T3_ABSXY,
            T4_INDY:    begin
                        // fetch {BAH,LL}, assuming no carry
                        adl_src = ADDR_ADD;
                        adh_src = ADDR_DATA;
                        if (ipage_up) begin
                            // low address overflowed, need to increment ADH
                            sb_src = REG_Z;
                            ci = 1;
                        end else begin
                            skip = 1;
                            save = store;
                        end
                        end

            T4_ABSXY,
            T5_INDY:    begin
                        adl_src = ADDR_HOLD;      // fetch {HH,LL} with incremented HH
                        adh_src = ADDR_ADD;
                        save = store;
                        end

            T2_XIND:    begin
                        adl_src = ADDR_DATA;      // fetch {0,BAL}
                        adh_src = ADDR_Z;
                        end                
            T3_XIND:    begin
                        adl_src = ADDR_ADD;     // fetch LL = {0,BAL+X}
                        adh_src = ADDR_Z;

                        // LL+1
                        sb_src = REG_ADD;
                        ci = 1;
                        end
            T4_XIND:    begin
                        holdalu = 1;    // hold LL in add
                        adl_src = ADDR_ADD;     // fetch {0,LL}
                        adh_src = ADDR_Z;
                        end
            T5_XIND:    begin
                        adl_src = ADDR_ADD;      // fetch {BAH,LL}, a carry will wrap LL without incrementing HH
                        adh_src = ADDR_DATA;
                        save = store;
                        end

            T2_INDY:    begin
                        // fetch low base addr  in zero page
                        adl_src = ADDR_DATA;
                        adh_src = ADDR_Z;
                        // increment ZPG pointer to fetch high base addr next cycle 
                        sb_src = REG_Z;
                        ci = 1;
                        end

            T3_INDY:    begin
                        // fetch high base addr in zero page
                        adl_src = ADDR_ADD;
                        adh_src = ADDR_Z;
                        // meanwhile, we have loaded BAL and are computing BAL+Y...
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
                        inc_pc = 1;
                        // add pcl + offset
                        adl_bi = 1;
                        sb_src = REG_DATA;
                        end

            T3_BRANCH:  begin
                        adl_src = ADDR_ADD;
                        adh_src = ADDR_HOLD;
                        jump = 1;              // addr -> pc
                        if (bpage_up) begin
                            // increment PCH
                            db_src = DB_PCH;
                            sb_src = REG_Z;
                            ci = 1;
                        end else if (bpage_down) begin
                            // decrement PCH
                            db_src = DB_PCH;
                            sb_src = REG_Z;
                            ai_inv = 1;
                        end else begin
                            // pch is valid, this cycle is our T0_FETCH fetch
                            inc_pc = 1;         
                            sync = 1;
                            skip = 1;
                        end
                        end

            T4_BRANCH: begin
                        adl_src = ADDR_HOLD;
                        adh_src = ADDR_ADD;
                        jump = 1;              // addr -> pc
                        inc_pc = 1;            // nextpc = pc++
                        sync = 1;
                        end

            T5_RTS,
            T3_JUMP:     begin
                        adl_src = ADDR_ADD;     // fetch new pc
                        adh_src = ADDR_DATA;
                        jump = 1;               // update pc
                        sync = !jump_ind;       //
                        end

            T2_PUSH:   begin
                            save = 1;
                            push = 1;
                        end

            T2_POP:     pop = 1;
            T3_POP:     begin end //fetch data...

            T2_BRK:     begin
                        //push pch
                        db_src = DB_PCH;
                        db_write = 1;
                        push = 1;
                        end

            T3_BRK:    begin
                        //push pcl
                        db_src = DB_PCL;
                        db_write = 1;
                        push = 1;
                        end

            T4_BRK:    begin
                        //push p
                        db_src = DB_P;
                        db_write = 1;
                        push = 1;
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
                            db_p = 1;
                            pop = 1;
                        end
            T4_RTS,
            T5_RTI:     sb_src = REG_Z; // store pcl in alu, fetch pch



            T2_JSR:     sb_src = REG_Z;      // alu nop: hold ADL in alu
            T3_JSR:     begin
                        holdalu = 1; // continue to hold ADL in alu
                        //push pch
                        db_src = DB_PCH;
                        db_write=1;                        
                        push = 1;
                        end

            T4_JSR:    begin
                        holdalu = 1; // continue to hold ADL in alu
                        //push pcl
                        db_src = DB_PCL;
                        db_write=1;                        
                        push = 1;
                        end
            T5_JSR:    begin
                        holdalu = 1; // continue to hold ADL in alu
                        // load ADH
                        end

            T_BOOT:     inc_pc = 1; // fetch low reset addr


            default:    jam = 1;
        endcase

        if (push || rpop) begin
            adl_src = ADDR_STACK;
            adh_src = ADDR_STACK;
        end

        if (exec) begin
            //configure the alu and bus for this instruction
            alu_OP = op_alu_OP;
            ai_inv = op_ai_inv;
            bi_inv = op_bi_inv;
            ci = op_carry_in == CARRY_C ? p[0] : op_carry_in[0];
            alu_mask = op_alu_mask;
            sb_src = op_sb_src;
            db_src = op_db_src;
        end

        //save result to register or memory
        if (save) begin
            sb_src = alu_en ? REG_ADD : op_sb_src;
            db_src = db_src == DB_P ? DB_P : DB_SB;
            case (op_dst)
                REG_DATA:   db_write = 1;
                REG_A:      sb_a=1;
                REG_X:      sb_x=1;
                REG_Y:      sb_y=1;
                REG_P:      db_p=1;
            endcase
        end

    end

endmodule

`timescale 1ns/1ps
`include "6502_defs.vh"

// 6502 uses level-triggered logic with latches that are updated during clk=1
// FPGA uses edge-triggered logic with registers loaded on posedge(clk)
// thus latches loaded during clk_m1 phase should be implemented as registers triggered on posedge(m2) and vice versa 

// unlike 6502 latches, registers are not transparent, cannot forward updated state on same clock,
// so several in several cases we will need special logic

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

module core_6502 #(
    parameter NMI_VECTOR=16'hfffa,
    parameter RST_VECTOR=16'hfffc,
    parameter IRQ_VECTOR=16'hfffe,
    parameter INITIAL_STATUS=FL_I | FL_U,
    parameter INITIAL_STACK=8'hfd
    )
    (
    input  logic clk_m1,
    input  logic clk_m2,
    input  logic rst,
    input  logic [7:0] data_i,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,

    output logic [15:0] addr,
    output logic [7:0] dor,
    output logic RW,
    output logic sync,
    output logic jam

    `ifdef DEBUG_REG
    ,input  logic reg_set_en,
    input  logic [15:0] pc_set,
    input  logic [7:0] s_set,
    input  logic [7:0] a_set,
    input  logic [7:0] x_set,
    input  logic [7:0] y_set,
    input  logic [7:0] p_set,
    output  logic [15:0] pc_dbg,
    output  logic [7:0] s_dbg,
    output  logic [7:0] a_dbg,
    output  logic [7:0] x_dbg,
    output  logic [7:0] y_dbg,
    output  logic [7:0] p_dbg
    `endif 

    );


    // ready signal
    wire rdy = READY | ~RW; //ignore not ready when writing

    // registers
    logic [7:0] ir, add;
    logic [7:0] a, s, x, y, p /*verilator public_flat*/;

      // state
    logic [7:0] state, next_state;

   // control signals
    logic [2:0] adl_src,adh_src;
    logic update_addrl,update_addrh;
    // logic store, load, rmw;
    logic ipc;
    logic inc_pc;
    logic skip;
    logic exec, exec_write, write_mem;
    logic jump, handle_int;
    logic holdalu;
    // logic ipage_up, bpage_up, bpage_down;

    // i/o data registers
    logic [7:0] data, push_data;
    always_ff @(posedge clk_m1) begin
        if (rst) begin
            data <= 0;
            dor <= 0;
            write_mem <= 0;
        end else begin
            // TODO: when to we need this?
            // if(rdy) data <= data_i;
            data <= data_i;


            if(exec & mem_wr) dor <= rmw ? alu_out : sb_result;
            else if(push) dor <= push_data;
            else dor <= data_i;
            write_mem <= write_back | push | (exec & mem_wr);

            // if(store)           dor <= sb_result;
                // else if(rmw)        dor <= alu_out;
            // end
    
        end
    end

    // assign dor = db;
    assign RW = !write_mem;

    // ADDRESS BUS
    // external address bus updated on m1
    always @(posedge clk_m1 ) begin
        if (rst) begin
            addr <= 0;
        end else begin
            if (update_addrh) addr[15:8] <= adh;
            if (update_addrl) addr[7:0] <= adl;
            if(reg_set_en) addr <= pc_set;
        end
    end
    // assign addr = {adh, adl};
    logic [7:0] adl,adh;
    always @(*) begin
        case(adl_src)
            ADDR_DATA: adl = data_i;
            ADDR_ALU: adl = add;
            ADDR_Z: adl = 8'b0;
            ADDR_INT: adl = rst_event ? RST_VECTOR[7:0] :
                            nmi_event ? NMI_VECTOR[7:0] :
                            IRQ_VECTOR[7:0];
            ADDR_STACK: adl = s;
            ADDR_HOLD: adl = addr[7:0];
            default: adl = pcl; //ADDR_PC
        endcase
        case(adh_src)
            ADDR_DATA: adh = data_i;
            ADDR_ALU: adh = add;
            ADDR_Z: adh = 8'b0;
            ADDR_INT: adh = rst_event ? RST_VECTOR[15:8] :
                            nmi_event ? NMI_VECTOR[15:8] :
                            IRQ_VECTOR[15:8];
            ADDR_STACK: adh = STACKPAGE;
            ADDR_HOLD: adh = addr[15:8];
            default: adh = pch; //ADDR_PC
        endcase
    end

    // PC (updated on m1)
    logic [7:0] pcl, pch;   // low and high byte of *incremented* pc
    logic [15:0] pc, pc_next /*verilator public_flat*/;
    always_comb begin
        pc_next = pc;
        if (inc_pc)     pc_next = pc + 1;
        else            pc_next = pc;

        if (read_pcl)   pc_next[7:0] = db;
        if (read_pch)   pc_next[15:8] = db;

        pcl = pc_next[7:0];
        pch = pc_next[15:8];
    end

    always @(posedge clk_m1 ) begin
        if (rst) begin
            pc <= RST_VECTOR;
        end else begin

            if ((nmi_event || irq_event) && sync) begin
                pc <= pc;
            end else if (jump) begin
                pc <= {adh, adl};
            end else begin
                pc <= pc_next;
            end

        `ifdef DEBUG_REG
            if(reg_set_en) begin
                pc <= pc_set;
            end
        `endif
        end
    end

    //instruction pointer: pc of current opcode
    //unused by core, but helpful for debug
    (* mark_debug = "true" *)  logic [15:0] ip;
    always @(posedge clk_m1 ) begin
        if (rst) ip <= RST_VECTOR;
        else if (sync && rdy) ip <= addr;
    end

    logic nmi_event, nmi_handled, irq_event, rst_event;
    logic fetch_intr;
    wire IRQ_masked = IRQ && !p[2];
    always @(posedge clk_m1 ) begin
        if (rst) begin
            nmi_event <= 0;
            irq_event <= 0;
            rst_event <= 1;
            nmi_handled <= 0;
            fetch_intr <= 0;
        end else begin

            rst_event <= 0;

            nmi_event <= NMI && !nmi_handled;
            if (IRQ_masked)
                irq_event <= 1;
            
            if(handle_int) begin
                nmi_handled <= nmi_event;
                nmi_event <= 0;
                irq_event <= 0;
                rst_event <= 0;
            end

            if(!NMI) nmi_handled <= 0;

            // set ir to BRK (0) rather than fetched instruction
            fetch_intr <= nmi_event | irq_event | rst_event;
        end
    end

    // opcode fetch and interrupt injection
    always @(posedge clk_m1 ) begin
        if (rst) ir <= 0;  //break from RESET_VECTOR
        else if (sync && rdy) ir <= data_i;

        `ifdef DEBUG_REG
            if (reg_set_en) ir <= 8'hea; // set ir to nop while we load debug state
        `endif         
    end

    // predecode flag for two-cycle ops (LD/CP, imm, impl)
    logic two_cycle;
    assign two_cycle =  (data_i ==? 8'b1??_000_?0) ||                            
                        (data_i ==? 8'b???_010_?? && data_i !=? 8'b0??_010_00) ||
                        (data_i ==? 8'b???_110_?0);

    // decode instruction
    logic [4:0] op_type;
    logic [6:0] db_src, db_src_exec, db_dst;
    logic [6:0] sb_src, sb_src_exec, sb_dst;
    logic alu_en;
    logic upNZ, upV, upC, bit_op;
    logic mem_rd, mem_wr, single_byte, idx_XY;
    logic take_branch;
    logic [7:0] set_mask, clear_mask;
    decode u_decode(
        .opcode         (ir),
        .pstatus        (p),
        .op_type        (op_type ),
        .db_src         (db_src_exec),
        .db_dst         (db_dst),
        .sb_src         (sb_src_exec),
        .sb_dst         (sb_dst),
        .alu_op         (alu_op_exec),
        .alu_en         (alu_en),
        .upNZ           (upNZ),
        .upV            (upV),
        .upC            (upC),
        .bit_op         (bit_op),
        .mem_rd         (mem_rd),
        .mem_wr         (mem_wr),
        .take_branch    (take_branch),
        .single_byte    (single_byte),
        .idx_XY         (idx_XY),
        .stack          (stack),
        .stack_ap       (stack_ap),
        .set_mask       (set_mask),
        .clear_mask     (clear_mask)
    );

    // internal buses
    // the real 6502 updates bus states on subcycles using out of phase clocks m1,m2)
    // e.g. when executing an alu operation on the first subcycle the sb bus carries an operand
    // and on the second subcycle the sb bus carries the result.
    // in order to represent the same timing with a single clock, we implement two sets of busses
    logic [7:0] sb, sb_result, db, sb_r, db_r;
    always @(posedge clk_m1 ) begin
        sb_r <= sb;
        db_r <= db;
    end
    always_comb begin
        
        case(sb_src)
            SB_A :   sb = a;
            SB_X :   sb = x;
            SB_Y :   sb = y;
            SB_S :   sb = s;
            SB_ADD : sb = add;
            SB_ADH : sb = adh;
            SB_DB :  sb = db_r;
            default: sb = 0;
        endcase

        if (alu_en) sb_result = add;
        else begin
            case(sb_src_exec)
                SB_A :   sb_result = a;
                SB_X :   sb_result = x;
                SB_Y :   sb_result = y;
                SB_S :   sb_result = s;
                SB_ADD : sb_result = add;
                SB_ADH : sb_result = adh;
                SB_DB :  sb_result = data;
                default: sb_result = 0;
            endcase
        end
        
        db = data_i;
    end

    // stack
    logic push, pull, pull_r;
    logic [1:0] stack_addr, stack_addr_r;
    logic read_a, read_p, read_pcl, read_pch;
    logic stack, stack_ap;
    parameter STACK_A   = 2'h0;
    parameter STACK_P   = 2'h1;
    parameter STACK_PCL = 2'h2;
    parameter STACK_PCH = 2'h3;

    always @(posedge clk_m1 ) begin
        if (rst) begin
            pull_r <= 0;
            stack_addr_r <= 0;
            read_a <= 0;
            read_p <= 0;
            read_pcl <= 0;
            read_pch <= 0;
        end else begin
            pull_r <= pull;
            stack_addr_r <= stack_addr;
            if (pull_r) begin
                case(stack_addr_r)
                    STACK_A:    read_a <= 1;
                    STACK_P:    read_p <= 1;
                    STACK_PCL:  read_pcl <= 1;
                    STACK_PCH:  read_pch <= 1;
                    default:    begin end
                endcase
            end else begin
                read_a <= 0;
                read_p <= 0;
                read_pcl <= 0;
                read_pch <= 0;
            end

        `ifdef DEBUG_REG
            if (reg_set_en || debug_clear) begin
                pull_r <= 0;
                read_a <= 0;
                read_p <= 0;
                read_pcl <= 0;
                read_pch <= 0;
            end
        `endif 

        end
    end

    always_comb begin
        case(stack_addr)
            STACK_A:    push_data = a;
            STACK_P:    push_data = irq_event ? p : p | FL_BU; // set break flags unless irq
            STACK_PCH:  push_data = pch;
            STACK_PCL:  push_data = pcl;
        endcase
    end



    //alu
    logic [5:0] alu_op, alu_op_exec;
    // logic alu_cin, alu_cin_exec;
    logic [7:0] alu_ai, alu_bi, alu_out;
    logic aluV, aluC;
    logic adl_add, adh_add;

    // in real 6502, adl routed to alu_bi and adh is routed via sb, but this requires subcycle control...
    // we are also cheating here a bit by using the registered addr than adl/adh bus, but this accomplishes the same effect
    assign  alu_ai = adl_add ? addr[7:0] :
                     adh_add ? addr[15:8] :
                     sb;
    assign alu_bi = data; 

    alu u_alu(
        .op     (alu_op  ),
        .ai     (alu_ai  ),
        .bi     (alu_bi  ),
        .ci    (p[0]),
        .out    (alu_out),
        .aluV   (aluV),
        .aluC   (aluC)
    );

    always @(posedge clk_m2 ) begin
        if (rst) begin
            add <= 0;
            // aluC_reg <= 0;
        end
        else if (!holdalu) begin
            add <= alu_out;
            // aluC_reg <= aluC;
        end
    end

    // result
    logic [7:0] result;
    logic resultZ, resultN;
    assign resultZ = ~|sb_result;
    assign resultN = bit_op ? alu_bi[7] : sb_result[7];

    logic [7:0] alu_reg;
    always @(posedge clk_m1) alu_reg <= alu_out;

    // always @(*) begin
    //     // TODO: ugh do this differently...
    //     if      (write_p)        result = p | FL_B;
    //     else if (write_pcl)      result = pcl;
    //     else if (write_pch)      result = pch;
    //     else if (write_back)     result = data;
    //     else if (rmw)            result = alu_reg; //TODO: ugh!!
    //     else if (alu_en)         result = add; //TODO: ugh!!
    //     else if (db_src[0])      result = data;
    //     else if (db_src[1])          result = p;
    //     else                     result = sb;

    // end


    //update registers
    always @(posedge clk_m1 ) begin
        if (rst) begin
            a <= 0;
            x <= 0;
            y <= 0;
            s <= INITIAL_STACK;
            p <= INITIAL_STATUS;              //disable IRQ on reset
        end else if(rdy) begin

            if (exec) begin
                // update status flags
                p <= ~clear_mask & (set_mask  | p);
                if (upC) p[0] <= aluC;
                if (upV) p[6] <= aluV;
                if (bit_op) p[6] <= alu_bi[6];
                if (upNZ) begin
                    p[1] <= resultZ;
                    p[7] <= resultN;
                end

                case (sb_dst)
                    SB_A:      a <= sb_result;
                    SB_X:      x <= sb_result;
                    SB_Y:      y <= sb_result;
                    SB_S:      s <= sb_result;
                    default:    begin end
                endcase    

            end

            if (push | pull) s <= add;
            if (read_a) begin
                a <= db;
                p[1] <= ~|db;
                p[7] <= db[7];
            end
            if (read_p) begin
                p <= db;
                p[4] <= p[4]; //ignore break flag
            end

            // if (handle_int && (irq_event | nmi_event)) p[2] <= 1;   // set interrupt bit
            if (handle_int) p[2] <= 1;   // set interrupt bit

        end

        `ifdef DEBUG_REG
            if(reg_set_en) begin
                s <= s_set;
                a <= a_set;
                x <= x_set;
                y <= y_set;
                p <= p_set;     
            end
        `endif

        p[4] <= 0;                                              //bit 4 doesnt exist but always reports low
        p[5] <= 1;                                              //bit 5 doesnt exist but always reports high
    end

    //state machine
    logic Tlast;
    always @(posedge clk_m1 ) begin
        if (rst) begin
            state <= T1;
        end
        else if (rdy) begin
            state <= next_state;
            `ifdef DEBUG_REG
                if(reg_set_en) state <= T_DEBUG;
                if(state == T_DEBUG) state <= T1;
            `endif
        end
    end

    //TODO:
    wire op_jsr = op_type == OP_JSR;

    logic inc_addr, dec_addr, sum_addr, add_idx;
    // control

    logic aluC_reg, aluN_reg;
    logic write_back;
    logic bpage_up, bpage_down;
    always @(posedge clk_m1 ) begin
        aluN_reg <= alu_out[7];
        aluC_reg <= aluC;
    end

    logic rmw;
    logic [6:0] r_idx;

    logic store;

    wire rmw = mem_wr & alu_en;

    always @(*) begin
        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 0;
        holdalu     = 0;
        skip        = 0;
        exec        = 0;
        // write_mem = 0;
        jump        = 0;
        sync        = 0;
        handle_int  = 0;
        update_addrl = 1;
        update_addrh = 1;
        adl_add = 0;
        adh_add = 0;
        r_idx = idx_XY ? SB_X : SB_Y;

        write_back = 0;
        
        stack_addr = stack_ap ? STACK_A : STACK_P;
        push = 0;
        pull = 0;

        store = (sb_dst == SB_DB) & !alu_en;

        // default alu behavior: store data in alu register
        alu_op = ALU_SUM;
        db_src = DB_M;
        sb_src = SB_Z;

        bpage_up = pcl[7] & !data[7] & !alu_out[7]; // crossed to next page if base>127, offset>0, and result <= 127
        bpage_down = !pcl[7] & data[7] & alu_out[7]; // crossed to prev page if base<=127, offset<0, and result > 127

        // From: https://www.nesdev.org/wiki/Visual6502wiki/6502_Timing_States
        // The convention for presenting time states for instruction execution here
        // is a a little different from that supplied in the usual programming literature in two ways:
        // by time numbering and by first through last cycles listed.
        // The time numbering issue applies to matching up the time codes used in Appendix A
        // of "MCS6500 Microcomputer Family Hardware Manual", with the time states documented here.
        // "T0" in the manual matches with the states that have [T1] in them here (most often T+ [T1]).
        // The rest of the time codes in the hardware manual listings match up with those here after being incremented by one.

        // The second-to-last time code in each hardware manual listing corresponds to the T0-containing states here,
        // and the last code in each listing is the [T1] state again.         
        
        case (state)
            T0: begin
                // holdalu = !stack; //whats this for??
                inc_pc = !single_byte;
                // exec = mem_wr & (stack || !alu_en);
                // write_mem = mem_wr;
                next_state = T1;
                end
            T1: begin
                // execute prev op
                exec = !mem_wr ;
                // fetch next op
                sync = 1;
                inc_pc = 1;
                next_state = two_cycle ? T0T2 : T2;
                end
            T0T2: begin
                inc_pc = !single_byte;
                next_state = T1;
                end
            T2: begin
                next_state = T3;
                case(op_type)
                    OP_ZPG: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};
                        exec = store;
                        next_state = T0;
                    end
                    OP_ZXY, OP_XIN, OP_INY: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};
                    end
                    OP_ABS, OP_AXY, OP_JUM, OP_JIN: begin
                        inc_pc = 1;
                    end
                    OP_BRA: begin
                        inc_pc = 1;
                        next_state = take_branch ? T3 : T1;
                    end
                    OP_PUS: begin
                        push = 1;
                        next_state = T0;
                    end
                    OP_PUL: begin
                        pull = 1;
                    end
                    OP_BRK: begin
                        inc_pc = 1;
                        stack_addr = STACK_PCH;
                        push = 1;
                    end
                    OP_JSR: begin
                        inc_pc = 1;
                        // point next addr at stack
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
                        adl_add = 1;
                        // load adl into stack 
                    end
                    default:
                        next_state = T_JAM; //not implemented yet
                endcase
                end
            T3: begin
                next_state = T4;
                case(op_type)
                    OP_ZXY: begin
                        sb_src = r_idx;
                        alu_op = ALU_SUM;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                        if(!rmw) begin
                            next_state = T0;
                            exec = store;
                        end
                    end
                    OP_XIN: begin
                        sb_src = r_idx;
                        alu_op = ALU_SUM;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                    end
                    OP_INY: begin
                        sb_src = r_idx;
                        alu_op = ALU_INB;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                    end
                    OP_ABS: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        if(!rmw) begin
                            next_state = T0;
                            exec = store;
                        end
                    end
                    OP_AXY: begin
                        sb_src = r_idx;
                        alu_op = ALU_SUM;
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        next_state = (!mem_wr & !aluC) ? T0 : T4;
                    end
                    OP_BRA: begin                        
                        adl_add = 1;
                        {adh_src, adl_src} = {ADDR_PC, ADDR_ALU};
                        if(bpage_up | bpage_down) begin
                            next_state = T4; //carry adh
                        end else begin
                            next_state = T1;
                        end
                        jump = 1;
                    end
                    OP_JUM: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        jump = 1;
                        next_state = T1;
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                    end
                    OP_PUL: begin
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
                        next_state = T0;
                    end
                    OP_BRK: begin
                        stack_addr = STACK_PCL;
                        push = 1;
                    end
                    OP_JSR: begin
                        stack_addr = STACK_PCH;
                        push = 1;

                        // // stack ptr to adl
                        // {adh_src, adl_src} = {ADDR_STACK, ADDR_HOLD};
                        // // read new adl into stack pointer
                        // // alu_ctl = {REG_M, REG_Z, 1'b0}; //TODO: make all read_* use db like this...
                        // // alu_op = ALU_ORA;
                        // // up_s = 1;
                    end
                    default:
                        next_state = T_JAM; //not implemented yet
                endcase
                end
            T4: begin
                next_state = T5;
                case(op_type)
                    OP_ZXY, OP_ABS:  // rmw fetch
                    begin
                        update_addrl = 0;
                        update_addrh = 0;
                        write_back = 1;
                    end
                    OP_AXY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};
                        alu_op = aluC_reg ? ALU_INB : ALU_ORA;
                        if(!rmw) begin
                            next_state = T0;
                            exec = store;
                        end
                    end
                    OP_XIN: begin
                        sb_src = SB_ADD;
                        alu_op = ALU_INC;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                    end
                    OP_INY: begin
                        sb_src = r_idx;
                        alu_op = ALU_SUM;
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        next_state = (!mem_wr & !aluC) ? T0 : T5;
                    end
                    OP_BRA: begin
                        adh_add = 1; 
                        alu_op = aluN_reg ? ALU_DEC : ALU_INC; //inc or dec
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_PC};
                        jump = 1;
                        next_state = T1;
                    end
                    OP_JIN: begin
                        // alu_ctl = {REG_Z, R_ALU, 1'b1};
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_ALU};
                    end
                    OP_BRK: begin
                        stack_addr = STACK_P;
                        push = 1;
                    end
                    OP_JSR: begin
                        stack_addr = STACK_PCL;
                        push = 1;
                        // // decrement addr (push stack)
                        // write_pch = 1;
                        // {adh_src, adl_src} = {ADDR_STACK, ADDR_ALU};

                        

                        // alu_ctl = {RE_NZ, R_ADL, 1'b0};
                    end
                    default:
                        next_state = T_JAM; //not implemented yet
                endcase
                end
            T5: begin
                next_state = T6;
                case(op_type)
                    OP_ZXY, OP_ABS: begin
                        update_addrl = 0;
                        update_addrh = 0;
                        exec = 1;
                        next_state = T0;
                    end
                    OP_AXY: begin
                        update_addrl = 0;
                        update_addrh = 0;
                        write_back = 1;
                    end
                    OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        next_state = T0;
                        exec = store;
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};
                        alu_op = aluC_reg ? ALU_INB : ALU_ORA;
                        next_state = T0;
                        exec = store;
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        jump = 1;
                        next_state = T1;
                    end
                    OP_BRK: begin
                        {adh_src, adl_src} = {ADDR_INT, ADDR_INT};
                        jump = 1;
                        inc_pc = 1;
                    end
                    OP_JSR: begin
                        // decrement addr (push stack)
                        // write_pcl = 1;
                        // alu_ctl = {RE_NZ, R_ADL, 1'b0};
                        // {adh_src, adl_src} = {ADDR_STACK, ADDR_ALU};
                    end
                    default:
                        next_state = T_JAM; //not implemented yet
                endcase
                end                
            T6: begin
                next_state = T7;
                case(op_type)
                    OP_AXY: begin
                        update_addrl = 0;
                        update_addrh = 0;
                        next_state = T0;
                        exec = 1;
                    end
                    OP_BRK: begin
                        jump = 1;
                        inc_pc = 1;
                        next_state = T7;
                    end
                    OP_JSR: begin
                        //read in ADH, and restore ADL from stack
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_STACK}; 
                        // restore stack pointer from ADL
                        // alu_ctl = {REG_S, R_ADL, 1'b0};
                        // up_s = 1;
                        jump = 1;
                        next_state = T1;
                    end
                    default:
                        next_state = T_JAM; //not implemented yet
                endcase
                end      
                T7: begin
                    next_state = T_JAM;
                    case(op_type)
                        OP_BRK: begin
                            {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                            jump = 1;
                            handle_int = 1; // TODO: ?
                            next_state = T1;
                        end
                        default:
                            next_state = T_JAM; //not implemented yet
                    endcase
                    end      
    
                default: begin
                next_state = T_JAM;
                // $display("6502 jammed at pc=0x%4h", pc);
            end                
        endcase

        // if (write_pcl | write_pch | write_p | write_back) write_mem=1;

        if (push) begin // TODO: set db_src?
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
            alu_op = ALU_DEC;
            sb_src = SB_S;
        end
        else if (pull)  begin
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
            alu_op = ALU_INC;
            sb_src = SB_S;
        end
        
        if (exec & alu_en) begin
            alu_op = alu_op_exec;
            db_src = db_src_exec;
            sb_src = sb_src_exec;
            // write_mem = mem_wr & !alu_en;
        end
        
        // if (write_back) begin
        //     sb_src = SB_Z;
        //     db_src = DB_M;
        //     write_mem = 1;
        // end else if (write_mem & alu_en) begin
        //     // rmw writeback
        //     sb_src = SB_ADD;
        //     db_src = DB_SB;
        // end

        // write_mem = write_pcl | write_pch | write_p | write_back | (mem_rd & exec & mem_wr);
        

        //     // for rts/rti, initial states are popping things off the stack
        //     // pops take 3 cycles to complete, so initial states are common...
        //     T2_RTS,                             // T3_RTS: pull pcl
        //     T3_RTS,                             // T4_RTS: fetch pcl, pull pch
        //     T2_RTI,                             // T3_RTI: pull p
        //     T3_RTI:                             // T4_RTI: fetch p, pull pcl
        //                 pull = 1;

        //     T4_RTI:     begin
        //                 db_p = 1;               //store p, fetch pcl, pull pch
        //                 pull = 1;
        //                 end
        //     T4_RTS,
        //     T5_RTI:     sb_src = REG_Z;         // store pcl in alu, fetch pch

        //     T5_RTS:    begin
        //                 adl_src = ADDR_ALU;     // fetch new pc
        //                 adh_src = ADDR_DATA;    
        //                 jump = 1;               // update pc from address
        //                 end


        //     // JSR
        //     // T1:          [pc] fetch op
        //     // T2:          [pc+1] fetch addr_low
        //     // T3:          [s], stash addr_low in s
        //     // T4:          [s], push pch
        //     // T5:          [s-1], push pcl
        //     // T6:          [pc+2], fetch addr_high
        //     // T0:          [addr], fetch next opcode

        //     // T2_JSR:     begin
        //     //             // fetch low addr (pc+1)
        //     //             // point at stack (on bus in T3)
        //     //             // adl <- s
        //     //             // adh <- 0
        //     //             end

        //     T2_JSR:     begin
        //                 inc_pc=0; // hold pc

        //                 // stash addr_low in s (db -> sb -> s)
        //                 sb_src=REG_DATA;
        //                 sb_s=1;

        //                 // addr stack
        //                 dec_addr=1;
        //                 end

        //     T3_JSR:     begin
        //                 dec_addr=1;

        //                 // holdalu = 1;            // continue to hold pcl in alu
        //                 // db_src = DB_PCH;        // push old pch
        //                 // db_write=1;                        
        //                 // push = 1;
        //                 end


        //     // push:       begin
        //     //             adl_src = ALU;
        //     //             adh_src = 0;
        //     //             alu_src = adl;
        //     //             alu_op = DEC;
        //     //             end

        //     // T4_JSR:     begin
        //     //             //alu -> adl
        //     //             //write_mem pch (push)
        //     //             //set up decrement adl on alu (push)
        //     //             end

        //     // T5_JSR:    begin
        //     //             //alu -> adl
        //     //             //write_mem pcl (push)
        //     //     holdalu = 1;            // continue to hold new pcl in alu
        //     //             db_src = DB_PCL;        // push old pcl
        //     //             db_write=1;                        
        //     //             push = 1;
        //     //             end
        //     // T6_JSR:     begin
        //     //                                     // load new pch
        //     //             holdalu = 1;            // continue to hold new pcl in alu
        //     //             end


        //     // T3_JSR:     begin
        //     //             sb_src = REG_Z;         // read new pcl and store in alu (compute pcl+0)
        //     //             end

        //     T4_JSR:    begin
        //                 holdalu = 1;            // continue to hold new pcl in alu
        //                 db_src = DB_PCL;        // push old pcl
        //                 // db_write=1;                        
        //                 push = 1;
        //                 end
        //     T5_JSR:     begin
        //                                         // load new pch
        //                 holdalu = 1;            // continue to hold new pcl in alu
        //                 end
        //     // state then transfers to T4_JUMP, where {pch, pcl} -> pc

        //     `ifdef DEBUG_REG
        //         T1_DEBUG: begin
        //                     sync = 1;
        //                 end
        //     `endif

        //     default:    jam = 1;
        // endcase

        // if (push || rpop) begin
        //     adl_src = ADDR_STACK;
        //     adh_src = ADDR_STACK;
        // end

        `ifdef DEBUG_REG
            if (reg_set_en || debug_clear) begin
                exec = 0;
                push = 0;
                pull = 0;
                // write_mem = 0;
            end
        `endif 
    end


    `ifdef DEBUG_CPU
        `include "debug/debug.sv"
    `endif 
    logic debug_clear;
    `ifdef DEBUG_REG
        always_comb begin
            pc_dbg = pc;
            s_dbg = s;
            a_dbg = a;
            x_dbg = x;
            y_dbg = y;
            p_dbg = p;
        end
        int cycle;
        always_ff @(posedge clk_m1) begin
            debug_clear <= 0;
            cycle <= cycle+1;
            if (state==T_DEBUG) begin
                debug_clear <= 1;
                cycle <= 0;
            end
        end
    `elsif
        assign debug_clear=0;
    `endif 


endmodule

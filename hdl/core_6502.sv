`timescale 1ns/1ps
`include "6502_defs.vh"

module core_6502 #(
    parameter NMI_VECTOR=16'hfffa,
    parameter RST_VECTOR=16'hfffc,
    parameter IRQ_VECTOR=16'hfffe,
    parameter INITIAL_STATUS=FL_I | FL_U,
    parameter INITIAL_STACK=8'hfd
    )
    (
    input  logic clk,
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
    logic [7:0] Tstate;

   // control signals
    logic [2:0] adl_src,adh_src;
    logic inc_pc;
    logic exec, write_mem;
    logic jump, handle_int;
    logic hold_addr, hold_alu;

    // i/o data registers
    assign RW = !write_mem;
    logic [7:0] data, push_data;

    always_ff @(posedge clk) begin
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


            `ifdef DEBUG_REG
                if(reg_set_en || debug_clear) begin
                    write_mem <= 0;
                end
            `endif

        end
    end

    // ADDRESS BUS
    logic [7:0] adl,adh;
    always @(posedge clk ) begin
        if (rst) begin
            addr <= 0;
        end else begin
            if(!hold_addr) addr <= {adh, adl};
            if(reg_set_en) addr <= pc_set;
        end
    end
    always @(*) begin
        case(adl_src)
            ADDR_DATA: adl = data_i;
            ADDR_ALU: adl = alu_out;
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
            ADDR_ALU: adh = alu_out;
            ADDR_Z: adh = 8'b0;
            ADDR_INT: adh = rst_event ? RST_VECTOR[15:8] :
                            nmi_event ? NMI_VECTOR[15:8] :
                            IRQ_VECTOR[15:8];
            ADDR_STACK: adh = STACKPAGE;
            ADDR_HOLD: adh = addr[15:8];
            default: adh = pch; //ADDR_PC
        endcase
    end

    // PC
    logic [15:0] pc, pc_next /*verilator public_flat*/;

    // pcl/pch: low and high byte of *incremented* pc
    wire [7:0] pcl = pc_next[7:0];
    wire [7:0] pch = pc_next[15:8];

    always_comb begin
        pc_next = pc;
        if (inc_pc)     pc_next = pc + 1;
        if (read_pcl)   pc_next[7:0] = db;
        if (read_pch)   pc_next[15:8] = db;
    end

    always @(posedge clk ) begin
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

    // interrupt handling
    logic nmi_event, nmi_handled, irq_event, rst_event;
    logic fetch_intr;
    wire IRQ_masked = IRQ && !p[2];
    always @(posedge clk ) begin
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
    always @(posedge clk ) begin
        if (rst) ir <= 0;  //break from RESET_VECTOR
        else if (sync && rdy) ir <= data_i;

        `ifdef DEBUG_REG
            if (reg_set_en) ir <= 8'hea; // set ir to nop while we load debug state
        `endif         
    end

    // predecode flag for two-cycle ops (LD/CP, imm, impl)
    logic two_cycle;
    assign two_cycle =  sync && ((data_i ==? 8'b1??_000_?0) ||                            
                                (data_i ==? 8'b???_010_?? && data_i !=? 8'b0??_010_00) ||
                                (data_i ==? 8'b???_110_?0));

    // decode instruction
    logic [4:0] op_type;
    logic [6:0] db_src, db_src_exec, db_dst;
    logic [6:0] sb_src, sb_src_exec, sb_dst;
    logic alu_en;
    logic upNZ, upV, upC, bit_op;
    logic mem_rd, mem_wr, single_byte, idx_XY;
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
    always @(posedge clk ) begin
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
            // SB_ADH : sb = adh;
            SB_DB :  sb = db_r;
            default: sb = 0;
        endcase

        if (alu_en) sb_result = alu_out;
        else begin
            case(sb_src_exec)
                SB_A :   sb_result = a;
                SB_X :   sb_result = x;
                SB_Y :   sb_result = y;
                SB_S :   sb_result = s;
                SB_ADD : sb_result = alu_out;
                // SB_ADH : sb_result = adh;
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

    always @(posedge clk ) begin
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
            read_a <= 0;
            read_p <= 0;
            read_pcl <= 0;
            read_pch <= 0;
            if (pull_r) begin
                case(stack_addr_r)
                    STACK_A:    read_a <= 1;
                    STACK_P:    read_p <= 1;
                    STACK_PCL:  read_pcl <= 1;
                    STACK_PCH:  read_pch <= 1;
                    default:    begin end
                endcase
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
            default:    push_data = 0;
        endcase
    end

    //alu
    logic [5:0] alu_op, alu_op_exec;
    logic [7:0] alu_ai, alu_bi, alu_out;
    logic aluV, aluC, adl_add, adh_add;

    // in real 6502, adl is routed to alu_bi and adh is routed to alu_ai via sb, but this requires latches and subcycle control...
    // we are also cheating here a bit by using the registered addr than adl/adh bus, but this accomplishes the same effect
    assign  alu_ai = adl_add ? addr[7:0] :
                     adh_add ? addr[15:8] :
                     hold_alu ? add:
                     sb;
    assign alu_bi = hold_alu ? 0 : data; 

    alu u_alu(
        .op     (alu_op  ),
        .ai     (alu_ai  ),
        .bi     (alu_bi  ),
        .ci    (p[0]),
        .out    (alu_out),
        .aluV   (aluV),
        .aluC   (aluC)
    );

    always @(posedge clk ) begin
        if (rst) add <= 0;
        else add <= alu_out;
    end

    //update registers
    always @(posedge clk ) begin
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
                    p[1] <= ~|sb_result;
                    p[7] <= bit_op ? alu_bi[7] : sb_result[7];;
                end

                case (sb_dst)
                    SB_A:      a <= sb_result;
                    SB_X:      x <= sb_result;
                    SB_Y:      y <= sb_result;
                    SB_S:      s <= sb_result;
                    default:    begin end
                endcase    

            end

            if ((push | pull | up_s) & !hold_s) s <= alu_out;
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
    logic Tlast, toT1, toTrmw;
    always @(posedge clk ) begin
        if (rst) begin
            Tstate <= T1;
        end
        else if (rdy) begin

            Tstate <=   Tlast ? T0 :
                        toT1 ? T1 : 
                        two_cycle ? T0T2 :
                        toTrmw ? TRMW1 : 
                        Tstate >> 1;

            `ifdef DEBUG_REG
                if(reg_set_en) Tstate <= T_DEBUG;
                if(Tstate == T_DEBUG) Tstate <= T1;
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
    always @(posedge clk ) begin
        aluN_reg <= alu_out[7];
        aluC_reg <= aluC;
    end

    logic [6:0] r_idx;
    logic store;
    wire rmw = mem_wr & alu_en;
    logic jsr_push, hold_s, up_s;
    always @(*) begin
        Tlast       = 0;
        toT1        = 0;
        toTrmw      = 0;

        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 0;
        hold_alu    = 0;
        hold_addr   = 0;
        exec        = 0;
        write_back  = 0;
        jump        = 0;
        sync        = 0;
        handle_int  = 0;
        adl_add     = 0;
        adh_add     = 0;
        r_idx       = idx_XY ? SB_X : SB_Y;
        
        stack_addr = stack_ap ? STACK_A : STACK_P;
        push        = 0;
        pull        = 0;
        jsr_push    = 0;
        hold_s      = 0;
        up_s        = 0;

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
        // and the last code in each listing is the [T1] Tstate again.         
        
        case (Tstate)
            T0: begin
                inc_pc = !single_byte;
                end
            T1: begin
                exec = !mem_wr;     // execute prev op
                inc_pc = 1;         // fetch next op
                sync = 1;
                end
            T0T2: begin
                inc_pc = !single_byte;
                toT1 = 1;
                end
            T2: begin
                inc_pc = 1;
                case(op_type)
                    OP_ZPG: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};
                        inc_pc = 0;
                        // todo!!
                        if(rmw) toTrmw = 1;
                        else begin
                            Tlast = 1;
                            exec = store;
                        end
                    end
                    OP_ZXY, OP_XIN, OP_INY: begin
                        inc_pc = 0;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};
                    end
                    OP_BNT: begin
                        toT1 = 1;
                    end
                    OP_PUS: begin
                        inc_pc = 0;
                        push = 1;
                        Tlast = 1;
                    end
                    OP_PUL: begin
                        inc_pc = 0;
                        pull = 1;
                    end
                    OP_BRK: begin
                        stack_addr = STACK_PCH;
                        push = 1;
                    end
                    OP_JSR: begin
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
                    end
                    OP_RTS: begin
                        inc_pc = 0;
                        pull = 1;
                        stack_addr = STACK_PCL;
                    end
                    OP_RTI: begin
                        inc_pc = 0;
                        pull = 1;
                        stack_addr = STACK_P;
                    end
                    default: begin end
                endcase
                end
            T3: begin
                case(op_type)
                    OP_ZXY: begin
                        sb_src = r_idx;
                        alu_op = ALU_SUM;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                        if(rmw) toTrmw = 1;
                        else begin
                            Tlast = 1;
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
                        if(rmw) toTrmw = 1;
                        else begin
                            Tlast = 1;
                            exec = store;
                        end
                    end
                    OP_AXY: begin
                        sb_src = r_idx;
                        alu_op = ALU_SUM;
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        Tlast = !mem_wr & !aluC;
                    end
                    OP_BRA: begin                        
                        adl_add = 1;
                        {adh_src, adl_src} = {ADDR_PC, ADDR_ALU};
                        jump = 1;
                        toT1 = (!bpage_up & !bpage_down);
                    end
                    OP_JUM: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        jump = 1;
                        toT1 = 1;
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                    end
                    OP_PUL: begin
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
                        Tlast = 1;
                    end
                    OP_BRK: begin
                        stack_addr = STACK_PCL;
                        push = 1;
                    end
                    OP_JSR: begin
                        push = 1;// push pch onto stack (via pc)
                        stack_addr = STACK_PCH;
                        jsr_push = 1;   // but dont do a real push, set s to adl which is currently in alu
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_HOLD}; // hold addr at stack 
                    end
                    OP_RTS: begin
                        pull = 1;
                        stack_addr = STACK_PCH;
                    end
                    OP_RTI: begin
                        pull = 1;
                        stack_addr = STACK_PCL;
                    end
                    default: begin end
                endcase
                end
            T4: begin
                case(op_type)
                    OP_ZXY, OP_ABS:  // rmw fetch
                    begin
                        hold_addr = 1;
                        write_back = 1;
                    end
                    OP_AXY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};
                        alu_op = aluC_reg ? ALU_INB : ALU_ORA;
                        if(rmw) toTrmw = 1;
                        else begin
                            Tlast = 1;
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
                        Tlast = (!mem_wr & !aluC);
                    end
                    OP_BRA: begin
                        adh_add = 1; 
                        alu_op = aluN_reg ? ALU_DEC : ALU_INC; //inc or dec
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_PC};
                        jump = 1;
                        toT1 = 1;
                    end
                    OP_JIN: begin
                        alu_op = ALU_INC;
                        sb_src = SB_ADD;
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_ALU};
                    end
                    OP_BRK: begin
                        stack_addr = STACK_P;
                        push = 1;
                    end
                    OP_JSR: begin
                        stack_addr = STACK_PCL;
                        push = 1;
                        jsr_push = 1;
                        hold_s = 1;

                        // decrement addr (push stack)
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_ALU};
                        alu_op = ALU_DEC;
                        adl_add = 1;
                    end
                    OP_RTI: begin
                        pull = 1;
                        stack_addr = STACK_PCH;
                    end
                    OP_RTS: begin end
                    default: begin end
                endcase
                end
            T5: begin
                case(op_type)
                    OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        Tlast = 1;
                        exec = store;
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};
                        alu_op = aluC_reg ? ALU_INB : ALU_ORA;
                        Tlast = 1;
                        exec = store;
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        jump = 1;
                        toT1 = 1;
                    end
                    OP_BRK: begin
                        {adh_src, adl_src} = {ADDR_INT, ADDR_INT};
                        jump = 1;
                        inc_pc = 1;
                    end
                    OP_JSR: begin
                        // fetch adh
                        // decrement addr and save in alu
                        adl_add = 1;
                        alu_op = ALU_DEC;
                    end
                    OP_RTI: begin
                        adl_add = 1; // save adl in alu
                    end
                    OP_RTS: begin
                        jump = 1;
                        Tlast = 1;
                    end
                    default: begin end
                endcase
                end                
            T6: begin
                case(op_type)
                    OP_BRK: begin
                        jump = 1;
                        inc_pc = 1;
                    end
                    OP_JSR: begin
                        //read in ADH, and restore ADL from stack and jump to new addr
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_STACK}; 
                        jump = 1;
                        // restore stack from alu
                        hold_alu = 1;
                        up_s = 1;
                        toT1 = 1;
                    end
                    OP_RTI: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        jump = 1;
                        toT1 = 1;
                    end
                    default: begin end
                endcase
                end      
            T7: begin
                case(op_type)
                    OP_BRK: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        jump = 1;
                        handle_int = 1; // TODO: ?
                        toT1 = 1;
                    end
                    default: begin end
                endcase
                end      

            TRMW1: begin
                    hold_addr = 1;
                    write_back = 1;
                    end
            TRMW2: begin
                    hold_addr = 1;
                    exec = 1;
                    Tlast = 1;
                    end

            default: begin
                // $display("6502 jammed at pc=0x%4h", pc);
                end                
    
        endcase

        if (push & !jsr_push) begin
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
            alu_op = ALU_DEC;
            sb_src = SB_S;
        end
        else if (pull)  begin
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
            alu_op = ALU_INC;
            sb_src = SB_S;
        end
        else if (pull_r)  begin
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
        end
        
        if (exec & alu_en) begin
            alu_op = alu_op_exec;
            db_src = db_src_exec;
            sb_src = sb_src_exec;
            // write_mem = mem_wr & !alu_en;
        end
        
        `ifdef DEBUG_REG
            if (reg_set_en || debug_clear) begin
                exec = 0;
                push = 0;
                pull = 0;
            end
        `endif 
    end

    //instruction pointer: pc of current opcode
    //unused by core, but helpful for debug
    (* mark_debug = "true" *)  logic [15:0] ip;
    always @(posedge clk ) begin
        if (rst)                ip <= RST_VECTOR;
        else if (sync && rdy)   ip <= addr;
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
        always_ff @(posedge clk) begin
            debug_clear <= 0;
            cycle <= cycle+1;
            if (Tstate==T_DEBUG) begin
                debug_clear <= 1;
                cycle <= 0;
            end
        end
    `elsif
        assign debug_clear=0;
    `endif 


endmodule

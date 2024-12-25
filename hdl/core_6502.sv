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
    logic exec, rexec;
    logic write_reg, write_mem;
    logic jump, handle_int;
    logic holdalu;
    // logic ipage_up, bpage_up, bpage_down;

    // i/o data registers
    logic [7:0] data;
    always_ff @(posedge clk_m1) begin
        if (rst) begin
            data <= 0;
        end else begin
            data <= data_i;
            // TODO: when to we need this?
            // if(rdy) data <= data_i;
        end
    end

    always_comb begin
        RW = !write_mem;
        dor = write_mem ? result : data;
    end

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

        if (read_pcl)   pc_next[7:0] = data_i;
        if (read_pch)   pc_next[15:8] = data_i;

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
    logic [3:0] op_dst, op_dst_exec;
    logic [3:0] a_src, b_src, a_src_exec, b_src_exec;
    logic alu_en;
    logic upNZ, upV, upC, bit_op;
    logic mem_rd, mem_wr, single_byte, idx_XY;
    logic take_branch;
    logic [7:0] set_mask, clear_mask;
    decode u_decode(
        .opcode         (ir),
        .pstatus        (p),
        .op_type        (op_type ),
        .dst            (op_dst_exec),
        .a_src          (a_src_exec),
        .b_src          (b_src_exec),
        .alu_op         (alu_op_exec),
        .alu_en         (alu_en),
        .alu_cin        (alu_cin_exec),
        .upNZ           (upNZ),
        .upV            (upV),
        .upC            (upC),
        .bit_op         (bit_op),
        .mem_rd         (mem_rd),
        .mem_wr         (mem_wr),
        .take_branch    (take_branch),
        .single_byte    (single_byte),
        .idx_XY         (idx_XY),
        .set_mask       (set_mask),
        .clear_mask     (clear_mask)
    );

    // internal buses
    logic [7:0] sb, db;
    always @(*) begin
        case(a_src) //sb bus (registers)
            REG_D:   sb = data;
            RE_ND:   sb = ~data;
            REG_A:   sb = a;
            REG_X:   sb = x;
            REG_Y:   sb = y;
            REG_S:   sb = s;
            R_PCL:   sb = pcl;
            R_PCH:   sb = pch;
            R_ADL:   sb = adl;
            R_ADH:   sb = adh;
            R_ALU:   sb = add;
            RE_NZ:   sb = 8'hff;
            default: sb = 8'h0;
        endcase
        case(b_src) //db bus (data)
            REG_D:   db = data;
            RE_ND:   db = ~data;
            REG_P:   db = irq_event ? p : p | FL_BU; // set break flags unless irq 
            REG_A:   db = a;
            REG_X:   db = x;
            REG_Y:   db = y;
            REG_S:   db = s;
            R_ADL:   db = adl;
            R_ADH:   db = adh;
            R_PCL:   db = pcl;
            R_PCH:   db = pch;
            R_ALU:   db = add;
            RE_NZ:   db = 8'hff;
            default: db = 8'h0;
        endcase
    end

    //alu
    logic [2:0] alu_op, alu_op_exec;
    logic alu_cin, alu_cin_exec;
    logic [7:0] alu_out;
    logic aluV, aluC;
    alu u_alu(
        .op     (alu_op  ),
        .ai     (sb  ),
        .bi     (db  ),
        .ci    (alu_cin),
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
    always @(*) begin
        if      (write_p)        result = p | FL_B;
        else if (write_pcl)      result = pcl;
        else if (write_pch)      result = pch;
        else if (write_back)     result = data;
        else if (alu_en)         result = alu_out;
        else if (a_src == REG_Z) result = db;
        else                     result = sb;

        resultZ = ~|result;
        resultN = bit_op ? db[7] : result[7];
    end

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
                if (bit_op) p[6] <= db[6];
                if (upNZ) begin
                    p[1] <= resultZ;
                    p[7] <= resultN;
                end

                case (op_dst)
                    REG_A:      a <= result;
                    REG_X:      x <= result;
                    REG_Y:      y <= result;
                    REG_S:      s <= result;
                    REG_P:      p <= result;
                    default:    begin end
                endcase    
            end

            if (push | pull) s <= alu_out;
            if (read_p) p <= irq_event ? p : p | FL_B; // set break flags unless irq
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
                if(reg_set_en) state <= T1;
            `endif
        end
    end

    //TODO:
    wire op_jsr = op_type == OP_JSR;

    logic inc_addr, dec_addr, sum_addr, add_idx;
    // control

    // control of alu during non-exec cycles
    logic [8:0] alu_ctl;  // {a_src, b_src, cin}

    logic aluC_reg, aluN_reg;
    logic push, write_pcl, write_pch, write_p, write_back;
    logic pull, read_pcl, read_pch, read_p;
    logic bpage_up, bpage_down;
    always @(posedge clk_m1 ) begin
        aluN_reg <= alu_out[7];
        aluC_reg <= aluC;
    end

    logic rmw;
    logic [3:0] r_idx;
    always @(*) begin
        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 0;
        holdalu     = 0;
        skip        = 0;
        exec        = 0;
        jump        = 0;
        sync        = 0;
        handle_int  = 0;
        update_addrl = 1;
        update_addrh = 1;
        op_dst = REG_Z;
        rmw = mem_rd & mem_wr;
        r_idx = idx_XY ? REG_X : REG_Y;

        push = 0;
        pull = 0;
        write_p = 0;
        write_pcl = 0;
        write_pch = 0;
        write_back = 0;
        read_p = 0;
        read_pcl = 0;
        read_pch = 0;
        
        // default alu behavior: store data in alu register
        alu_op = ALU_ADD;
        alu_ctl = {REG_Z, REG_D, 1'b0};

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
                exec = mem_wr;
                inc_pc = !single_byte;
                next_state = T1;
                end
            T1: begin
                // execute prev op
                exec = !mem_wr;
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
                        push = 1; // initiate push pch
                    end
                    default:
                        next_state = T_JAM; //not implemented yet
                endcase
                end
            T3: begin
                next_state = T4;
                case(op_type)
                    OP_ZXY: begin
                        alu_ctl = {r_idx, REG_D, 1'b0};
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                        if(!rmw) next_state = T0;
                    end
                    OP_XIN: begin
                        alu_ctl = {r_idx, REG_D, 1'b0};
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                    end
                    OP_INY: begin
                        alu_ctl = {REG_Z, REG_D, 1'b1};
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                    end
                    OP_ABS: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        if(!rmw) next_state = T0;
                    end
                    OP_AXY: begin
                        alu_ctl = {r_idx, REG_D, 1'b0};
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        next_state = (!mem_wr & !aluC) ? T0 : T4;
                    end
                    OP_BRA: begin
                        // next_state = T1;
                        alu_ctl = {R_PCL, REG_D, 1'b0}; // add branch offset
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
                        write_pch = 1; // complete push pch 
                        push = 1; // initiate push pcl
                    end
                    default:
                        next_state = T_JAM; //not implemented yet
                endcase
                end
            T4: begin
                next_state = T5;
                case(op_type)
                    OP_ZXY:
                    begin
                        update_addrl = 0;
                        update_addrh = 0;
                    end
                    OP_ABS:
                    begin // rmw fetch
                        update_addrl = 0;
                        update_addrh = 0;
                    end
                    OP_AXY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};
                        alu_ctl = {REG_Z, REG_D, aluC_reg};
                        if(!rmw) next_state = T0;
                    end
                    OP_XIN: begin
                        alu_ctl = {REG_Z, R_ALU, 1'b1};
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                    end
                    OP_INY: begin
                        alu_ctl = {r_idx, REG_D, 1'b0};
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        next_state = (!mem_wr & !aluC) ? T0 : T5;
                    end
                    OP_BRA: begin
                        alu_ctl = aluN_reg ? {R_PCH, RE_NZ, 1'b0} : {R_PCH, REG_Z, 1'b1};
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_PC};
                        jump = 1;
                        next_state = T1;
                    end
                    OP_JIN: begin
                        alu_ctl = {REG_Z, R_ALU, 1'b1};
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_ALU};
                    end
                    OP_BRK: begin
                        write_pcl = 1; // complete push pcl
                        push = 1; // initiate push p
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
                        next_state = T0;
                        write_back = 1;
                    end
                    OP_AXY: begin
                        update_addrl = 0;
                        update_addrh = 0;
                    end
                    OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        next_state = T0;
                    end
                    OP_INY: begin
                        alu_ctl = {REG_Z, REG_D, aluC_reg};
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};
                        next_state = T0;
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        jump = 1;
                        next_state = T1;
                    end
                    OP_BRK: begin
                        write_p = 1; // complete push p
                        {adh_src, adl_src} = {ADDR_INT, ADDR_INT};
                        jump = 1;
                        inc_pc = 1;
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
                        write_back = 1;
                    end
                    OP_BRK: begin
                        jump = 1;
                        inc_pc = 1;
                        next_state = T7;
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

       
        if (push) begin // TODO: set db_src?
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
            {a_src, b_src, alu_cin} = {RE_NZ, R_ADL, 1'b0};
        end
        else if (pull)  begin
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
            {a_src, b_src, alu_cin} = {REG_Z, R_ADL, 1'b1};
        end
        else if (exec) begin
            {a_src, b_src, alu_cin} = {a_src_exec, b_src_exec, alu_cin_exec};
            op_dst = op_dst_exec; //todo: is this needed?
            alu_op = alu_op_exec;
        end
        else begin
            {a_src, b_src, alu_cin} = alu_ctl;
        end

        write_mem = write_pcl | write_pch | write_p | write_back | (exec & mem_wr);



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
            // if (reg_set_en) begin
            //     sync = 0;
            // end
            if (reg_set_en || debug_T1) begin
                exec = 0;
            end
        `endif 
    end


    `ifdef DEBUG_CPU
        `include "debug/debug.sv"
    `endif 
    logic debug_T1;
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
            debug_T1 <= reg_set_en;
            cycle <= reg_set_en ? 0 : cycle+1;
        end
    `elsif
        assign debug_T1=0;
    `endif 


endmodule

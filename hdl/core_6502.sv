`timescale 1ns/1ps
`include "6502_defs.vh"

module core_6502 #(
    parameter NMI_VECTOR = 16'hfffa,
    parameter RST_VECTOR = 16'hfffc,
    parameter IRQ_VECTOR = 16'hfffe,
    parameter A_RST      = 8'h0,
    parameter X_RST      = 8'h0,
    parameter Y_RST      = 8'h0,
    parameter S_RST      = 8'hff,
    parameter P_RST      = FL_I | FL_U
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

    );

    // ready signal
    wire rdy = READY | ~RW; //ignore not ready when writing

    // registers
    logic [7:0] ir /*verilator public*/;
    logic [7:0] add /*verilator public*/;
    logic [7:0] a /*verilator public*/;
    logic [7:0] s /*verilator public*/;
    logic [7:0] x /*verilator public*/;
    logic [7:0] y /*verilator public*/;
    logic [7:0] p /*verilator public*/;

      // state
    logic [7:0] Tstate /*verilator public*/;

   // control signals
    logic [2:0] adl_src,adh_src;
    logic inc_pc;
    logic write_mem;
    logic jump, handle_int;
    logic hold_alu;


    // ADDRESS BUS
    logic [7:0] adl,adh;
    always @(posedge clk ) begin
        if (rst) begin
            addr <= 0;
        end else begin
            addr <= {adh, adl};
        end
    end
    always @(*) begin
        case(adl_src)
            ADDR_DATA:  adl = db;
            ADDR_ALU:   adl = alu_out;
            ADDR_INT:   adl = rst_event ? RST_VECTOR[7:0] :
                              nmi_event ? NMI_VECTOR[7:0] :
                              IRQ_VECTOR[7:0];
            ADDR_STACK: adl = s;
            ADDR_HOLD:  adl = addr[7:0];
            default:    adl = pcl; //ADDR_PC
        endcase
        case(adh_src)
            ADDR_DATA:  adh = db;
            ADDR_ALU:   adh = alu_out;
            ADDR_Z:     adh = 8'b0;
            ADDR_INT:   adh = rst_event ? RST_VECTOR[15:8] :
                              nmi_event ? NMI_VECTOR[15:8] :
                              IRQ_VECTOR[15:8];
            ADDR_STACK: adh = STACKPAGE;
            ADDR_HOLD:  adh = addr[15:8];
            default:    adh = pch; //ADDR_PC
        endcase
    end


    // internal buses
    // the real 6502 updates bus states on subcycles using out of phase clocks m1,m2)
    // e.g. when executing an alu operation on the first subcycle the sb bus carries an operand
    // and on the second subcycle the sb bus carries the result.
    // in order to represent the same timing with a single clock, we implement two sets of busses
    logic [7:0] sb, sb_result, db, db_result;
    always_comb begin
        
        // "source" bus
        case(1'b1)
            src[0] : sb = a;
            src[1] : sb = x;
            src[2] : sb = y;
            src[3] : sb = s;
            src[4] : sb = add;
            src[5] : sb = data;
            default: sb = 0;
        endcase

        // "result" bus
        case(1'b1)
            alu_en  :       sb_result = alu_out;
            src_result[0] : sb_result = a;
            src_result[1] : sb_result = x;
            src_result[2] : sb_result = y;
            src_result[3] : sb_result = s;
            src_result[4] : sb_result = alu_out;
            src_result[5] : sb_result = data;
            default       : sb_result = 0;
        endcase
        
        // data read bus
        db = data_i;

        // data write bus
        case(1'b1)
            stack_push[0]:  db_result = a;
            stack_push[1]:  db_result = interrupt ? p : p | FL_BU; // set break flags unless irq
            stack_push[2]:  db_result = pcl;
            stack_push[3]:  db_result = pch;
            default         db_result = sb_result;
        endcase
    end

    // i/o data registers
    assign RW = !write_mem | rst_event;

    // TODO: this is a bit of a hack, and prob can be done organically..
    logic write_back;

    logic [7:0] data;
    always_ff @(posedge clk) begin
        if (rst) begin
            data <= 0;
            dor <= 0;
        end else begin
            // TODO: when to we need this?
            // if(rdy) data <= data_i;
            data <= data_i;
            dor <= write_back ? data_i : db_result;
        end
    end


    // PC
    logic [15:0] pc /*verilator public*/;
    logic [15:0] pc_next;

    // pcl/pch: low and high byte of *incremented* pc
    wire [7:0] pcl = pc_next[7:0];
    wire [7:0] pch = pc_next[15:8];

    always_comb begin
        pc_next = pc;
        if (inc_pc)          pc_next = pc + 1;   // increment pc
        if (stack_read[2])   pc_next[7:0] = db;  // pull pcl from stack
        if (stack_read[3])   pc_next[15:8] = db; // pull pch from stack
    end

    always @(posedge clk ) begin
        if (rst) begin
            pc <= 0;
        end else begin
            if ((nmi_event || irq_event) && sync) begin
                pc <= pc;
            end else if (jump) begin
                pc <= {adh, adl};
            end else begin
                pc <= pc_next;
            end

        end
    end

    // interrupt handling
    logic nmi_event, nmi_handled, irq_event, rst_event /*verilator public*/;
    // verilator lint_off SYMRSVDWORD
    wire interrupt = nmi_event || irq_event;
    // verilator lint_on SYMRSVDWORD

    logic fetch_intr;
    wire IRQ_masked = IRQ && !p[2];
    always @(posedge clk ) begin
        if (rst) begin
            nmi_event <= 0;
            irq_event <= 0;
            rst_event <= 1;
            nmi_handled <= 0;
            // fetch_intr <= 0;
        end else begin

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

        end
    end

    // set ir to BRK (0) rather than fetched instruction
    assign fetch_intr = sync && interrupt;

    // opcode fetch and interrupt injection
    always @(posedge clk ) begin
        if (rst || rst_event || fetch_intr) ir <= 0;  //break from RESET_VECTOR
        else if (sync && rdy) ir <= db;
    end

    // decode instruction
    logic [4:0] op_type;
    logic [5:0] src, src_result, dst_result;
    logic alu_en;
    logic [5:0] alu_op, alu_op_result;
    logic upNZ, upV, upC, bit_op;
    logic mem_rd, mem_wr, single_byte, idx_XY;
    logic stack_ap;
    logic [7:0] set_mask, clear_mask;
    decode u_decode(
        .opcode         (ir),
        .pstatus        (p),
        .op_type        (op_type ),
        .src            (src_result),
        .dst            (dst_result),
        .alu_op         (alu_op_result),
        .alu_en         (alu_en),
        .upNZ           (upNZ),
        .upV            (upV),
        .upC            (upC),
        .bit_op         (bit_op),
        .single_byte    (single_byte),
        .idx_XY         (idx_XY),
        .stack_ap       (stack_ap),
        .set_mask       (set_mask),
        .clear_mask     (clear_mask)
    );
    assign mem_rd = src_result[5];
    assign mem_wr = dst_result[5];

    // stack
    logic push, pull, push_r, pull_r;
    logic [3:0] stack_push, stack_push_r, stack_pull, stack_pull_r, stack_read;
    always @(posedge clk ) begin
        if (rst) begin
            pull_r <= 0;
            push_r <= 0;
            stack_pull_r <= 0;
            stack_push_r <= 0;
            stack_read <= 0;
        end else begin
            pull_r <= pull;
            push_r <= push;
            stack_pull_r <= stack_pull;
            stack_push_r <= stack_push;
            stack_read <= stack_pull_r;
        end
    end

    //alu
    logic [7:0] alu_ai, alu_bi, alu_out;
    logic aluV, aluC, adl_add, adh_add;

    // in real 6502, adl is routed to alu_bi and adh is routed to alu_ai via sb, controlled at subcycle
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
            a <= A_RST;
            x <= X_RST;
            y <= Y_RST;
            s <= S_RST;
            p <= P_RST;
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

                if(dst_result[0]) a <= sb_result;
                if(dst_result[1]) x <= sb_result;
                if(dst_result[2]) y <= sb_result;
                if(dst_result[3]) s <= sb_result;

            end

            if (up_s) s <= alu_out;
            if (stack_read[0]) begin //pull a from stack
                a <= db;
                p[1] <= ~|db;
                p[7] <= db[7];
            end
            if (stack_read[1]) begin //pull p from stack
                p <= db;
                p[4] <= p[4]; //ignore break flag
            end

            // if (handle_int && (irq_event | nmi_event)) p[2] <= 1;   // set interrupt bit
            if (handle_int) p[2] <= 1;   // set interrupt bit
        end

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
                        toTrmw ? TRMW1 : 
                        Tstate >> 1;

        end
    end
    assign jam = !|Tstate; //if Tstate reaches all zeros we have a jam

    logic inc_addr, dec_addr, sum_addr, add_idx;
    // control

    logic aluC_reg, aluN_reg;
    logic bpage_up, bpage_down;
    always @(posedge clk ) begin
        aluN_reg <= alu_out[7];
        aluC_reg <= aluC;
    end

    wire rmw = mem_wr & alu_en;
    wire store = mem_wr & !alu_en;
    // wire exec = mem_wr ? Tlast : Tstate[6];  // execute opcode on T1 or last 
    logic exec, exec_store;

    // always @(posedge clk ) begin
    //     exec <= exec_prep;
    // end


    logic jsr_push, up_s;
    logic [5:0] idx;
    always_comb begin
        Tlast       = 0;
        toT1        = 0;
        toTrmw      = 0;

        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 0;
        hold_alu    = 0;
        exec        = 0;
        exec_store  = 0;
        write_back  = 0;
        jump        = 0;
        sync        = 0;
        handle_int  = 0;
        adl_add     = 0;
        adh_add     = 0;
        idx       = idx_XY ? REG_X : REG_Y;
        
        write_mem   = 0;
        stack_push  = 0;
        stack_pull  = 0;
        push        = 0;
        pull        = 0;
        jsr_push    = 0;
        up_s        = 0;

        // store = mem_wr & !alu_en;

        // default alu behavior: store data in alu register
        alu_op = ALU_SUM;
        src = REG_Z;

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
                write_mem = mem_wr;
                end
            T1: begin
                exec = !mem_wr;
                inc_pc = 1;         // fetch next op
                sync = 1;
                end
            T2: begin
                inc_pc = 1;
                case(op_type)
                        OP_ZPG: begin
                            {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};
                            inc_pc = 0;
                            if(rmw) toTrmw = 1;
                            else begin
                                Tlast = 1;
                                // exec = store;
                            end
                        end
                        OP_ZXY, OP_XIN, OP_INY: begin
                            inc_pc = 0;
                            {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};
                        end
                        OP_PUS: begin
                            inc_pc = 0;
                            stack_push = stack_ap ? STACK_A : STACK_P;
                            Tlast = 1;
                        end
                        OP_PUL: begin
                            inc_pc = 0;
                            stack_pull = stack_ap ? STACK_A : STACK_P;
                        end
                        OP_BRK: begin
                            inc_pc = !interrupt; //only increment pc if this is actual BRK opcode, not external interrupt
                            stack_push = STACK_PCH;
                        end
                        OP_JSR: begin
                            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
                        end
                        OP_RTS: begin
                            inc_pc = 0;
                            stack_pull = STACK_PCL;
                        end
                        OP_RTI: begin
                            inc_pc = 0;
                            stack_pull = STACK_P;
                        end
                        OP_IMM,
                        OP_IMP,
                        OP_BNT: begin
                            // this is effectively the T0T2 state
                            inc_pc = !single_byte;
                            toT1 = 1;
                        end
                        default: begin end
                    endcase
                end
            T3: begin
                case(op_type)
                    OP_ZXY: begin
                        src = idx;
                        alu_op = ALU_SUM;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                        if(rmw) toTrmw = 1;
                        else begin
                            Tlast = 1;
                            // exec = store;
                        end
                    end
                    OP_XIN: begin
                        src = idx;
                        alu_op = ALU_SUM;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                    end
                    OP_INY: begin
                        src = idx;
                        alu_op = ALU_INB;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                    end
                    OP_ABS: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        if(rmw) toTrmw = 1;
                        else begin
                            Tlast = 1;
                            // exec = store;
                        end
                    end
                    OP_AXY: begin
                        src = idx;
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
                        stack_push = STACK_PCL;
                    end
                    OP_JSR: begin
                        stack_push = STACK_PCH; // push pch onto stack (via pc)
                        jsr_push = 1;           // but dont do math on stack pointer...
                        up_s = 1;               // instead set s directly to adl which is currently in alu
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_HOLD}; // hold addr at stack 
                    end
                    OP_RTS: begin
                        stack_pull = STACK_PCH;
                    end
                    OP_RTI: begin
                        stack_pull = STACK_PCL;
                    end
                    default: begin end
                endcase
                end
            T4: begin
                case(op_type)
                    OP_AXY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};
                        alu_op = aluC_reg ? ALU_INB : ALU_ORA;
                        if(rmw) toTrmw = 1;
                        else begin
                            Tlast = 1;
                            // exec = store;
                        end
                    end
                    OP_XIN: begin
                        src = REG_ADD;
                        alu_op = ALU_INC;
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};
                    end
                    OP_INY: begin
                        src = idx;
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
                        src = REG_ADD;
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_ALU};
                    end
                    OP_BRK: begin
                        stack_push = STACK_P;
                    end
                    OP_JSR: begin
                        stack_push = STACK_PCL;
                        jsr_push = 1;           // but dont do math on stack register...
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_ALU}; // decrement adl directly
                        adl_add = 1;
                        alu_op = ALU_DEC;
                    end
                    OP_RTI: begin
                        stack_pull = STACK_PCH;
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
                        // exec = store;
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};
                        alu_op = aluC_reg ? ALU_INB : ALU_ORA;
                        Tlast = 1;
                        // exec = store;
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
                        alu_op = ALU_DEC; // decrement addr and save in alu
                        adl_add = 1;      
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
                        hold_alu = 1;  // restore stack from alu
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
                    {adh_src, adl_src} = {ADDR_HOLD, ADDR_HOLD};
                    write_back = 1; //this is a bit of a hack
                                    // not needed if we can "naturally" load
                                    // db_result but with data_i

                    end
            TRMW2: begin
                    {adh_src, adl_src} = {ADDR_HOLD, ADDR_HOLD};
                    exec = 1;
                    Tlast = 1;
                    end

            default: begin
                // $display("6502 jammed at pc=0x%4h", pc);
                end                
    
        endcase

        push = |stack_push;
        pull = |stack_pull;

        if (push & !jsr_push) begin
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
            alu_op = ALU_DEC;
            src = REG_S;
            up_s = 1;
        end
        else if (pull)  begin
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
            alu_op = ALU_INC;
            src = REG_S;
            up_s = 1;
        end
        else if (pull_r)  begin
            {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
        end

        if (exec & alu_en) begin
            alu_op = alu_op_result;
            src = src_result;
        end

        if((exec & mem_wr) | push_r) write_mem = 1;
    end

    //instruction pointer: pc of current opcode
    //unused by core, but helpful for debug
    (* mark_debug = "true" *)  logic [15:0] ip;
    always @(posedge clk ) begin
        if (rst)                ip <= RST_VECTOR;
        else if (sync && rdy)   ip <= addr;
    end

    int cycle /*verilator public*/;
    always_ff @(posedge clk) begin
        if (rst) cycle <= 0;
        else cycle <= cycle+1;
    end

endmodule

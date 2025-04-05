`timescale 1ns/1ps
`include "6502_defs.vh"

// 6502 core with synchronous memory,
// output address bus appears one clock earlier than in a real 6502

module core #(
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
    output logic [15:0] addr,
    output logic [7:0] data_o,
    output logic rw,
    input  logic [7:0] data_i,
    input  logic ready,
    input  logic so,
    input  logic nmi,
    input  logic irq,
    output logic sync,
    output logic jam
    );

    wire rdy = ready | ~rw; //ignore not ready when writing

    // registers
    (* mark_debug = "true" *) logic [7:0] ir /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] add /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] a /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] s /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] x /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] y /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] p /*verilator public*/;

    // ADDRESS BUS
    (* mark_debug = "true" *) logic [7:0] adl, adh;
    logic [7:0] adl_r, adh_r;
    assign addr = {adh,adl};
    always @(posedge clk ) begin
        if (rst) begin
            adl_r <= 0;
            adh_r <= 0;
        end else begin
            adl_r <= adl;
            adh_r <= adh;
        end
    end
    always @(*) begin
        unique case(1'b1)
            adl_src[0]: adl = pcl;  // ADDR_PC
            adl_src[1]: adl = db;   // ADDR_DATA
            adl_src[2]: adl = add;  // ADDR_ALU
            adl_src[3]: adl = rst_event ? RST_VECTOR[7:0] :
                              nmi_event ? NMI_VECTOR[7:0] :
                              IRQ_VECTOR[7:0]; // ADDR_INT
            adl_src[4]: adl = s;    // ADDR_STACK
            adl_src[5]: adl = adl_r;// ADDR_HOLD
            default:    adl = 0;    // ADDR_Z
        endcase
    end
    always @(*) begin
        unique case(1'b1)
            adh_src[0]: adh = pch;  // ADDR_PC
            adh_src[1]: adh = db;   // ADDR_DATA
            adh_src[2]: adh = add;  // ADDR_ALU
            adh_src[3]: adh = rst_event ? RST_VECTOR[15:8] :
                              nmi_event ? NMI_VECTOR[15:8] :
                              IRQ_VECTOR[15:8]; // ADDR_INT
            adh_src[4]: adh = STACKPAGE; // ADDR_STACK
            adh_src[5]: adh = adh_r;// ADDR_HOLD
            default:    adh = 0;    // ADDR_Z
        endcase
    end

    // internal buses
    // the real 6502 updates bus states on subcycles using out of phase clocks m1,m2)
    // e.g. when executing an alu operation on the first subcycle the sb bus carries an operand
    // and on the second subcycle the sb bus carries the result.
    // in order to represent the same timing with a single clock, we implement two sets of busses
    logic [7:0] sb, sb_result, db, db_result;
    logic dummy_write;
    assign data_o = db_result;
    assign rw = !write_mem | rst | rst_event;
    logic [3:0] stack_push, stack_read; // one-hot control for push/pull registers

    // db read bus
    assign db = data_i;

    // sb "source" bus
    always_comb begin
        unique case(1'b1)
            sb_src[0] : sb = a;
            sb_src[1] : sb = x;
            sb_src[2] : sb = y;
            sb_src[3] : sb = s;
            sb_src[4] : sb = add;
            sb_src[5] : sb = db;
            sb_src[6] : sb = adh;
            default:    sb = 0;
        endcase
    end

    // sb "result" bus
    always_comb begin
        case(1'b1)
            alu_en,
            sb_src_exec[4] : sb_result = add;
            sb_src_exec[0] : sb_result = a;
            sb_src_exec[1] : sb_result = x;
            sb_src_exec[2] : sb_result = y;
            sb_src_exec[3] : sb_result = s;
            sb_src_exec[5] : sb_result = db;
            default:         sb_result = 0;
        endcase
    end        
    // db write bus
    always_comb begin
        unique case(1'b1)
            stack_push[0]:  db_result = a;
            stack_push[1]:  db_result = int_event ? p : p | FL_BU; // set break flag on push unless irq
            stack_push[2]:  db_result = pcl;
            stack_push[3]:  db_result = pch;
            dummy_write:    db_result = db;         // bit of a hack to match RMW behavior
            default:        db_result = sb_result;
        endcase
    end

    // PC
    (* mark_debug = "true" *) logic [15:0] pc /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] pch, pcl;   // low and high byte of next pc
    logic [15:0] pc_next;
    assign {pch, pcl} = pc_next;

    always_comb begin
        unique case(1'b1)
            inc_pc:         pc_next = pc+1;
            stack_read[2]:  pc_next = {pc[15:8], db};  // pull pcl from stack
            stack_read[3]:  pc_next = {db, pc[7:0]};   // pull pch from stack
            default:        pc_next = pc;
        endcase
    end
    always @(posedge clk ) begin
        if (rst) begin
            pc <= 0;
        end else begin
            pc <= jump ? addr : pc_next;
        end
    end

    // interrupt handling
    logic nmi_event, nmi_handled, irq_event, rst_event /*verilator public*/;
    // verilator lint_off SYMRSVDWORD
    wire int_event = nmi_event || irq_event;
    // verilator lint_on SYMRSVDWORD
    always @(posedge clk ) begin
        if (rst) begin
            nmi_event <= 0;
            irq_event <= 0;
            rst_event <= 1;
            nmi_handled <= 0;
        end else begin

            nmi_event <= nmi && !nmi_handled;
            if (irq && !p[2])
                irq_event <= 1;

            if(brk_int) begin
                nmi_handled <= nmi_event;
                nmi_event <= 0;
                irq_event <= 0;
                rst_event <= 0;
            end

            if(!nmi)
                nmi_handled <= 0;
        end
    end

    // opcode fetch and interrupt injection
    always @(posedge clk ) begin
        if (rst || rst_event || (sync && int_event)) ir <= 0;  //break from RESET_VECTOR
        else if (sync && rdy) ir <= db;
    end

    // decode instruction
    logic [4:0] op_type;
    logic [6:0] sb_src, sb_src_exec, sb_src_ctrl, dst;
    logic alu_en;
    logic [8:0] alu_op_exec;
    logic [3:0] alu_flags_ctrl;
    logic single_byte, idx_XY;
    logic stack_ap, and_op, bit_op, sl_op, sr_op;
    logic clc, cli, clv, cld, sec, sei, sed;
    logic [7:0] result_mask;
    decode u_decode(
        .op         (ir),
        .op_type        (op_type ),
        .src            (sb_src_exec),
        .dst            (dst),
        .alu_op         (alu_op_exec),
        .alu_en         (alu_en),
        .single_byte    (single_byte),
        .idx_XY         (idx_XY),
        .stack_ap       (stack_ap),
        .and_op         (and_op),
        .bit_op         (bit_op),
        .sl_op          (sl_op),
        .sr_op          (sr_op),
        .clc            (clc),
        .cli            (cli),
        .clv            (clv),
        .cld            (cld),
        .sec            (sec),
        .sei            (sei),
        .sed            (sed),
        .result_mask    (result_mask)
    );

    //alu
    logic [8:0] alu_op;
    logic [7:0] alu_ai, alu_bi;
    logic adl_add, adh_add, alu_az, sb_db;
    logic sumC, sumV;
    assign alu_ai = adl_add ? adl :
                    alu_az ? 0 :
                    sb;
    assign alu_bi = sb_db ? sb : db;

    alu u_alu(
        .clk    (clk),
        .rst    (rst),
        .op     (alu_op),
        .ai     (alu_ai),
        .bi     (alu_bi),
        .ci     (p[0]),
        .out    (add),
        .sumC   (sumC),
        .sumV   (sumV),
        .bpage  (bpage)
    );

    // branch logic
    logic take_branch, bpage;
    always_comb begin
        unique case(ir[7:6])
            2'b00:   take_branch = p[7] ^ !ir[5]; // BPL, BMI
            2'b01:   take_branch = p[6] ^ !ir[5]; // BVC, BVS
            2'b10:   take_branch = p[0] ^ !ir[5]; // BCC, BCS
            2'b11:   take_branch = p[1] ^ !ir[5]; // BNE, BEQ
        endcase
    end

    //registers
    logic so_r, exec, alu_rdy, sb_s;
    logic [7:0] a_next, x_next, y_next, s_next;
    wire result_rdy = (exec && !alu_en) || alu_rdy;
    // update registers
    always_comb begin
        a_next = (result_rdy & dst[0]) ? sb_result :
                    stack_read[0] ? db : a;
        x_next = (result_rdy & dst[1]) ? sb_result : x;
        y_next = (result_rdy & dst[2]) ? sb_result : y;
        s_next = (result_rdy & dst[3]) ? sb_result :
                    sb_s ? sb : s;
    end

    // update p status register
    logic [7:0] p_next;
    wire dbz = (db==0);
    wire sbz = (sb_result==0);
    always_comb begin
        p_next = stack_read[1] ? db : p;

        if ( result_rdy ) begin
            if(result_mask[7]) p_next[7] = sb_result[7];
            if(result_mask[6]) p_next[6] = sumV;
            if(result_mask[1]) p_next[1] = sbz;
            if(result_mask[0]) p_next[0] = sumC;
        end

        if (exec) begin
            if (clc) p_next[0] = 0;
            if (cli) p_next[2] = 0;
            if (clv) p_next[6] = 0;
            if (cld) p_next[3] = 0;
            if (sec) p_next[0] = 1;
            if (sei) p_next[2] = 1;
            if (sed) p_next[3] = 1;

            // there are a few other special cases where alu status is
            // not updated with alu result (result_rdy) but
            // directly with alu input (exec):
            if(and_op | bit_op | stack_read[0]) begin
                p_next[1] = dbz;
                p_next[7] = db[7];
            end
            if(bit_op) p_next[6] = db[6];
            if(sr_op) p_next[0] = sb[0]; // shift-right carry out
        end

        if (so & !so_r) p_next[6] = 1;  //set overflow on re of SO pin
        if (brk_int) p_next[2] = 1;     //set interrupt bit on BRK

        p_next[4] = 0;                  //bit 4 doesnt exist but always reports low
        p_next[5] = 1;                  //bit 5 doesnt exist but always reports high
    end

    always @(posedge clk ) begin
        if (rst) begin
            a <= A_RST;
            x <= X_RST;
            y <= Y_RST;
            s <= S_RST;
            p <= P_RST;
            so_r <= 0;
            alu_rdy <= 0;
        end else if(rdy) begin

            alu_rdy <= exec & alu_en;
            a <= a_next;
            x <= x_next;
            y <= y_next;
            s <= s_next;
            p <= p_next;
            so_r <= so;
        end
    end

    // control state machine
    (* mark_debug = "true" *) logic [5:0] adl_src,adh_src;
    (* mark_debug = "true" *) logic inc_pc;
    (* mark_debug = "true" *) logic write_mem;
    (* mark_debug = "true" *) logic jump, brk_int;
    (* mark_debug = "true" *) logic hold_alu;    
    control u_control(
        .clk            (clk),
        .rst            (rst),
        .rdy            (rdy),
        .op_type        (op_type),
        .wr_op          (dst[5]),
        .alu_en         (alu_en),
        .single_byte    (single_byte),
        .stack_ap       (stack_ap),
        .int_event      (int_event),
        .aluC           (sumC),
        .aluN           (add[7]),
        .idx_XY         (idx_XY),
        .bpage          (bpage),
        .take_branch    (take_branch),
        .sl_op          (sl_op),
        .sync           (sync),
        .exec           (exec),
        .alu_flags      (alu_flags_ctrl),
        .sb_src         (sb_src_ctrl),
        .inc_pc         (inc_pc),
        .adl_src        (adl_src),
        .adh_src        (adh_src),
        .write_mem      (write_mem),
        .dummy_write    (dummy_write),
        .jump           (jump),
        .brk_int     (brk_int),
        .adl_add        (adl_add),
        .alu_az        (alu_az),
        .stack_push     (stack_push),
        .stack_read     (stack_read),
        .sb_s           (sb_s),
        .sb_db          (sb_db)
    );

    assign sb_src = exec ? sb_src_exec : sb_src_ctrl;
    assign alu_op = exec ? alu_op_exec : {alu_flags_ctrl, ALU_SUM};

    //below are not used internally but helpful for debug

    //instruction pointer: pc of current opcode
    (* mark_debug = "true" *)  logic [15:0] ip;
    always @(posedge clk ) begin
        if (rst)                ip <= RST_VECTOR;
        else if (sync && rdy)   ip <= pc;
    end

    int cycle /*verilator public*/;
    always_ff @(posedge clk) begin
        if (rst) cycle <= 0;
        else cycle <= cycle+1;
    end

    (* mark_debug = "true" *) logic [9:0] Tstate /*verilator public*/;
    assign Tstate = u_control.Tstate;
    assign jam = Tstate==0; //if Tstate reaches all zeros we have a jam

endmodule

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
    logic [7:0] ir /*verilator public*/;
    logic [7:0] add /*verilator public*/;
    logic [7:0] a /*verilator public*/;
    logic [7:0] s /*verilator public*/;
    logic [7:0] x /*verilator public*/;
    logic [7:0] y /*verilator public*/;
    logic [7:0] p /*verilator public*/;

    // ADDRESS BUS
    logic [7:0] adl, adh, adl_r, adh_r;
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
        case(adl_src)
            ADDR_DATA:  adl = db;
            ADDR_ALU:   adl = add;
            ADDR_INT:   adl = rst_event ? RST_VECTOR[7:0] :
                              nmi_event ? NMI_VECTOR[7:0] :
                              IRQ_VECTOR[7:0];
            ADDR_STACK: adl = s;
            ADDR_HOLD:  adl = adl_r;
            default:    adl = pcl;
        endcase
        case(adh_src)
            ADDR_DATA:  adh = db;
            ADDR_ALU:   adh = add;
            ADDR_Z:     adh = 8'b0;
            ADDR_INT:   adh = rst_event ? RST_VECTOR[15:8] :
                              nmi_event ? NMI_VECTOR[15:8] :
                              IRQ_VECTOR[15:8];
            ADDR_STACK: adh = STACKPAGE;
            ADDR_HOLD:  adh = adh_r;
            default:    adh = pch;
        endcase
    end


    // internal buses
    // the real 6502 updates bus states on subcycles using out of phase clocks m1,m2)
    // e.g. when executing an alu operation on the first subcycle the sb bus carries an operand
    // and on the second subcycle the sb bus carries the result.
    // in order to represent the same timing with a single clock, we implement two sets of busses
    logic [7:0] sb, sb_result, db, db_result;
    logic dummy_write;
    assign db = data_i;
    assign data_o = db_result;
    assign rw = !write_mem | rst | rst_event;
    logic [3:0] stack_push, stack_read; // one-hot control for push/pull registers
    always_comb begin

        // sb "source" bus
        case(1'b1)
            sb_src[0] : sb = a;
            sb_src[1] : sb = x;
            sb_src[2] : sb = y;
            sb_src[3] : sb = s;
            sb_src[4] : sb = add;
            sb_src[5] : sb = db;
            default: sb = 0;
        endcase

        // sb "result" bus
        case(1'b1)
            alu_en,
            sb_src_exec[5],
            sb_src_exec[4] : sb_result = add;
            sb_src_exec[0] : sb_result = a;
            sb_src_exec[1] : sb_result = x;
            sb_src_exec[2] : sb_result = y;
            sb_src_exec[3] : sb_result = s;
            default        : sb_result = 0;
        endcase
        
        // data write bus
        case(1'b1)
            stack_push[0]:  db_result = a;
            stack_push[1]:  db_result = p_push;
            stack_push[2]:  db_result = pcl;
            stack_push[3]:  db_result = pch;
            dummy_write:    db_result = db;         // bit of a hack to match RMW behavior
            default         db_result = sb_result;
        endcase
    end

    // PC
    logic [15:0] pc /*verilator public*/;
    logic [15:0] pc_next = pc + 1;
    logic [7:0] pch, pcl;   // low and high byte of next pc
    always_comb begin
        {pch, pcl} = pc;
        case(1'b1)
            inc_pc:         {pch, pcl} = pc_next;
            stack_read[2]:  pcl = db;   // pull pcl from stack
            stack_read[3]:  pch = db;   // pull pch from stack
            default         begin end
        endcase
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
                pc <= {pch, pcl};
            end
        end
    end

    // interrupt handling
    logic nmi_event, nmi_handled, irq_event, rst_event /*verilator public*/;
    // verilator lint_off SYMRSVDWORD
    wire interrupt = nmi_event || irq_event;
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

            if(handle_int) begin
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
        if (rst || rst_event || (sync && interrupt)) ir <= 0;  //break from RESET_VECTOR
        else if (sync && rdy) ir <= db;
    end

    // decode instruction
    logic [4:0] op_type;
    logic [5:0] sb_src, sb_src_exec, dst;
    logic alu_en;
    logic [5:0] alu_op, alu_op_exec;
    logic upNZ, upV, upC;
    logic single_byte, idx_XY;
    logic stack_ap;
    logic [7:0] alu_mask, set_mask, clear_mask;
    decode u_decode(
        .opcode         (ir),
        .pstatus        (p),
        .op_type        (op_type ),
        .src            (sb_src_exec),
        .dst            (dst),
        .alu_op         (alu_op_exec),
        .alu_en         (alu_en),
        .single_byte    (single_byte),
        .idx_XY         (idx_XY),
        .stack_ap       (stack_ap),
        .alu_mask       (alu_mask),
        .set_mask       (set_mask),
        .clear_mask     (clear_mask)
    );

    //alu
    logic [7:0] alu_ai, alu_status;
    logic adl_add, adh_add, bpage;
    assign  alu_ai = adl_add ? adl :
                     adh_add ? adh :
                     sb;
    alu u_alu(
        .clk    (clk),
        .rst    (rst),
        .op     (alu_op  ),
        .ai     (alu_ai  ),
        .bi     (db  ),
        .ci     (p[0]),
        .out    (add),
        .status (alu_status),
        .bpage  (bpage)
    );

    //registers
    logic so_r, exec, exec_r, up_s;
    always @(posedge clk ) begin
        if (rst) begin
            a <= A_RST;
            x <= X_RST;
            y <= Y_RST;
            s <= S_RST;
            p <= P_RST;
            so_r <= 0;
            exec_r <= 0;
        end else if(rdy) begin

            exec_r <= exec;

            if (exec_r) begin
                if(dst[0]) a <= sb_result;
                if(dst[1]) x <= sb_result;
                if(dst[2]) y <= sb_result;
                if(dst[3]) s <= sb_result;
                p <= ~clear_mask & (set_mask | (~alu_mask & p) | (alu_mask & alu_status));
            end

            so_r <= so;
            if (so & !so_r) p[6] <= 1; //set overflow on so rising edge

            if (up_s) s <= add;
            if (stack_read[0]) begin //pull a from stack
                a <= db;
                p[1] <= ~|db;
                p[7] <= db[7];
            end
            if (stack_read[1]) begin //pull p from stack
                p <= db;
                p[4] <= p[4]; //ignore break flag
            end

            if (handle_int) p[2] <= 1;   // set interrupt bit
        end

        p[4] <= 0;                                              //bit 4 doesnt exist but always reports low
        p[5] <= 1;                                              //bit 5 doesnt exist but always reports high
    end
    wire [7:0] p_push = interrupt ? p : p | FL_BU; // set break flag on push unless irq

    // state machine
    logic [7:0] Tstate /*verilator public*/;
    logic [2:0] adl_src,adh_src;
    logic inc_pc;
    logic write_mem;
    logic jump, handle_int;
    logic hold_alu;    
    control u_control(
        .clk            (clk),
        .rst            (rst),
        .rdy            (rdy),
        .op_type        (op_type),
        .wr_op          (dst[5]),
        .alu_en         (alu_en),
        .single_byte    (single_byte),
        .stack_ap       (stack_ap),
        .interrupt      (interrupt),
        .alu_status     (alu_status),
        .alu_op_exec    (alu_op_exec),
        .sb_src_exec    (sb_src_exec),
        .idx_XY         (idx_XY),
        .bpage          (bpage),
        .Tstate         (Tstate),
        .sync           (sync),
        .exec           (exec),
        .alu_op         (alu_op),
        .sb_src         (sb_src),
        .inc_pc         (inc_pc),
        .adl_src        (adl_src),
        .adh_src        (adh_src),
        .write_mem      (write_mem),
        .dummy_write    (dummy_write),
        .jump           (jump),
        .handle_int     (handle_int),
        .adl_add        (adl_add),
        .adh_add        (adh_add),
        .stack_push     (stack_push),
        .stack_read     (stack_read),
        .up_s           (up_s),
        .jam            (jam)
    );

    //below are unused but helpful for debug

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

endmodule

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
    assign data_o = db_result;
    assign rw = !write_mem | rst | rst_event;
    logic [3:0] stack_push, stack_read; // one-hot control for push/pull registers

    // db read bus
    assign db = data_i;

    // sb "source" bus
    always_comb begin
        case(1'b1)
            sb_src[0] : sb = a;
            sb_src[1] : sb = x;
            sb_src[2] : sb = y;
            sb_src[3] : sb = s;
            sb_src[4] : sb = add;
            sb_src[5] : sb = db;
            sb_src[6] : sb = adh;
            default: sb = 0;
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
            default: sb_result = 0;
        endcase
    end        
    // db write bus
    always_comb begin
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
    (* mark_debug = "true" *) logic [15:0] pc /*verilator public*/;
    logic [15:0] pc_next = pc + 1;
    (* mark_debug = "true" *) logic [7:0] pch, pcl;   // low and high byte of next pc
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
    logic [6:0] sb_src, sb_src_exec, sb_src_ctrl, dst;
    logic alu_en;
    logic [5:0] alu_op_exec, alu_op_ctrl;
    logic single_byte, idx_XY;
    logic stack_ap, bit_op, shift_op;
    logic clc, cli, clv, cld, sec, sei, sed;
    logic [7:0] result_mask;
    decode u_decode(
        .opcode         (ir),
        .op_type        (op_type ),
        .src            (sb_src_exec),
        .dst            (dst),
        .alu_op         (alu_op_exec),
        .alu_en         (alu_en),
        .single_byte    (single_byte),
        .idx_XY         (idx_XY),
        .stack_ap       (stack_ap),
        .bit_op         (bit_op),
        .shift_op       (shift_op),
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
    logic [5:0] alu_op;
    logic [7:0] alu_ai, alu_bi;
    logic alu_ci;
    logic adl_add, adh_add;
    logic shiftC, sumC, sumV;
    assign alu_ai = adl_add ? adl : sb;
                    // adh_add ? adh :
    assign alu_bi = db;
    assign alu_ci = p[0];

    alu u_alu(
        .clk    (clk),
        .rst    (rst),
        .op     (alu_op),
        .ai     (alu_ai),
        .bi     (alu_bi),
        .ci     (alu_ci),
        .out    (add),
        .shiftC (shiftC),
        .sumC   (sumC),
        .sumV   (sumV)
    );

    // branch logic
    logic take_branch, bpage;
    always_comb begin
        case(ir[7:6])
            2'h0:   take_branch = p[7] ^ !ir[5]; // BPL, BMI
            2'h1:   take_branch = p[6] ^ !ir[5]; // BVC, BVS
            2'h2:   take_branch = p[0] ^ !ir[5]; // BCC, BCS
            2'h3:   take_branch = p[1] ^ !ir[5]; // BNE, BEQ
        endcase
    end
    logic [7:0] db_r;  //TODO clean up?
    always @(posedge clk ) db_r <= db;
    assign bpage = (db_r[7] == add[7]) && (db_r[7] ^ adl_r[7]); // branch crosses page?

    //registers
    logic so_r, exec, alu_rdy, sb_s;
    logic [7:0] a_next, x_next, y_next, s_next, p_next, p_push, result_status;
    always_comb begin
        a_next = a;
        x_next = x;
        y_next = y;
        s_next = sb_s ? sb : s;
        p_next = p;

        result_status = p;
        if(result_mask[7]) result_status[7] = sb_result[7];
        if(result_mask[6]) result_status[6] = sumV;
        if(result_mask[1]) result_status[1] = sb_result==0;
        if(result_mask[0]) result_status[0] = sumC;
        if ( exec && !alu_en || alu_rdy ) begin
            if(dst[0]) a_next = sb_result;
            if(dst[1]) x_next = sb_result;
            if(dst[2]) y_next = sb_result;
            if(dst[3]) s_next = sb_result;
            p_next = result_status;
        end

        // some special cases with fussy visual6502 timing...
        if (exec) begin
            if(bit_op) begin
                p_next[7:6] = db[7:6];
                p_next[1] = (db==0);
            end
            if(shift_op) begin
                p_next[0] = shiftC;
            end
        end
        if(bit_op & sync) p_next[1] = (db_r & a)==0;


        if (so & !so_r) p_next[6] = 1;  //set overflow on so rising edge
        if (stack_read[0]) begin        //pull a from stack
            a_next = db;
            p_next[1] = db == 0;
            p_next[7] = db[7];
        end
        if (stack_read[1]) begin        //pull p from stack
            p_next = db;
        end
        if (handle_int) p_next[2] = 1;  // set interrupt bit
        p_next[4] = 0;                  //bit 4 doesnt exist but always reports low
        p_next[5] = 1;                  //bit 5 doesnt exist but always reports high

        p_push = interrupt ? p : p | FL_BU; // set break flag on push unless irq
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

            if (exec) begin
                if (clc) p[0] <= 0;
                if (cli) p[2] <= 0;
                if (clv) p[6] <= 0;
                if (cld) p[3] <= 0;
                if (sec) p[0] <= 1;
                if (sei) p[2] <= 1;
                if (sed) p[3] <= 1;
            end

            so_r <= so;
        end
    end

    // state machine
    (* mark_debug = "true" *) logic [7:0] Tstate /*verilator public*/;
    (* mark_debug = "true" *) logic [2:0] adl_src,adh_src;
    (* mark_debug = "true" *) logic inc_pc;
    (* mark_debug = "true" *) logic write_mem;
    (* mark_debug = "true" *) logic jump, handle_int;
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
        .interrupt      (interrupt),
        .aluC           (sumC),
        .aluN           (add[7]),
        .idx_XY         (idx_XY),
        .bpage          (bpage),
        .take_branch    (take_branch),
        .Tstate         (Tstate),
        .sync           (sync),
        .exec           (exec),
        .alu_op         (alu_op_ctrl),
        .sb_src         (sb_src_ctrl),
        .inc_pc         (inc_pc),
        .adl_src        (adl_src),
        .adh_src        (adh_src),
        .write_mem      (write_mem),
        .dummy_write    (dummy_write),
        .jump           (jump),
        .handle_int     (handle_int),
        .adl_add        (adl_add),
        // .adh_add        (adh_add),
        .stack_push     (stack_push),
        .stack_read     (stack_read),
        .sb_s           (sb_s),
        .jam            (jam)
    );

    assign sb_src = exec ? sb_src_exec : sb_src_ctrl;
    assign alu_op = exec ? alu_op_exec : alu_op_ctrl;

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

`timescale 1ns / 1ps
`include "6502_defs.vh"

module core6502 #(
    parameter NMI_VECTOR      = 16'hfffa,
    parameter RST_VECTOR      = 16'hfffc,
    parameter IRQ_VECTOR      = 16'hfffe,
    parameter A_RST           = 8'h0,
    parameter X_RST           = 8'h0,
    parameter Y_RST           = 8'h0,
    parameter S_RST           = 8'hff,
    parameter P_RST           = FL_IU,
    parameter PC_RST          = 16'h0,
    parameter MEM_REG         = 1, // is data_i already registered on clk?
    parameter WRITE_ON_NOTRDY = 0 // real 6502 ingores rdy on write cycles. Disable this when using external mem that might stall and lose writes.
) (
    input  logic        clk,
    input  logic        rst,
    output logic [15:0] addr,
    output logic [ 7:0] data_o,
    output logic        rw,
    input  logic [ 7:0] data_i,
    input  logic        ready,
    input  logic        so,
    input  logic        nmi,
    input  logic        irq,
    output logic        sync,
    output logic        jam
);

    logic [7:0] dl;
    generate
        if (MEM_REG) begin : gen_datai_direct
            assign dl = data_i;
        end 
        else begin : gen_datai_reg
            always_ff @(posedge clk ) dl <= rst ? '0 : data_i;
        end
    endgenerate

    assign rdy = ready | (WRITE_ON_NOTRDY & ~rw);

    // registers
    (* mark_debug = "true" *) logic [7:0] ir  /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] add  /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] a  /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] s  /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] x  /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] y  /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] p  /*verilator public*/;
    (* mark_debug = "true" *) logic [7:0] pch, pcl  /*verilator public*/;

    logic so_r, so_re, rdy;
    always_ff @(posedge clk) if(rdy) so_r <= so;
    assign so_re = rdy & so & ~so_r; // rising edge of so

    // PC
    (* mark_debug = "true" *) logic [15:0] pc  /*verilator public*/;
    logic [15:0] pc_next;
    assign pc_next = inc_pc ? pc + 1 : pc;
    assign pch     = db_pull[PULL_PCH] ? dl : pc_next[15:8];
    assign pcl     = db_pull[PULL_PCL] ? dl : pc_next[7:0];

    always_ff @(posedge clk) begin
        if (rst) begin
            pc <= PC_RST;
        end else if(rdy) begin
            if (int_event && sync) begin
                pc <= pc;
            end else if (jump) begin
                pc <= {adh, adl};
            end else begin
                pc <= {pch, pcl};
            end
        end
    end

    // internal buses
    (* mark_debug = "true" *) logic [7:0] sb, db;
    logic [2:0] sb_src, sb_dst;
    logic [5:0] db_src;
    logic [3:0] db_pull;
    wire        dbz = (db == 0);
    wire        sbz = (sb == 0);

    // memory bus
    (* mark_debug = "true" *) logic [7:0] adl, adh;
    (* mark_debug = "true" *) logic [15:0] addr_r;
    assign rw     = !wr_en | rst | rst_event;
    assign data_o = rw ? '0 : db;
    assign addr   = {adh, adl};

    // registers
    always_ff @(posedge clk) begin
        if (rst) begin
            a      <= A_RST;
            x      <= X_RST;
            y      <= Y_RST;
            s      <= S_RST;
            p      <= P_RST;
            addr_r <= '0;

        end else if (rdy) begin
            addr_r <= addr;

            case (sb_dst)
                SB_A:   a <= sb;
                SB_X:   x <= sb;
                SB_Y:   y <= sb;
                SB_S:   s <= sb;
                default: ;
            endcase
            if (db_pull[PULL_A]) a <= db;

            p    <= db_pull[PULL_P] ? db : p_update;
            p[4] <= 0;  //bit 4 doesnt exist but always reports low
            p[5] <= 1;  //bit 5 doesnt exist but always reports high
        end
    end

    // sb bus (mainly registers and alu output)
    always_comb begin
        case (sb_src)
            SB_A:   sb = a;
            SB_X:   sb = x;
            SB_Y:   sb = y;
            SB_S:   sb = s;
            SB_ADD: sb = add;
            SB_DATA:   sb = dl;
            SB_PCH: sb = pc[15:8];
            default: sb = 0;
        endcase
    end

    // db bus (mainly memory and pull/push regs)
    always_comb begin
        case (db_src)
            DB_DATA: db = dl;
            DB_A:    db = a;
            DB_P:    db = int_event ? p : p | FL_BU;  // set break flag on push unless irq
            DB_PCL:  db = pcl;
            DB_PCH:  db = pch;
            DB_SB:   db = sb;
            default: db = '0;
        endcase
    end

    // address bus
    always_comb begin
        case (adl_src)
            ADDR_PC:    adl = pcl;
            ADDR_DATA:  adl = dl;
            ADDR_ALU:   adl = add;
            ADDR_INT:   adl =   rst_event ? RST_VECTOR[7:0] :
                                nmi_event ? NMI_VECTOR[7:0] :
                                IRQ_VECTOR[7:0];
            ADDR_STACK: adl = s;
            ADDR_HOLD:  adl = addr_r[7:0];
            default:    adl = 0;  //ADDR_Z
        endcase

        case (adh_src)
            ADDR_PC:    adh = pch;
            ADDR_DATA:  adh = dl;
            ADDR_ALU:   adh = add;
            ADDR_INT:   adh =   rst_event ? RST_VECTOR[15:8] :
                                nmi_event ? NMI_VECTOR[15:8] :
                                IRQ_VECTOR[15:8];
            ADDR_STACK: adh = STACKPAGE;
            ADDR_HOLD:  adh = addr_r[15:8];
            default:    adh = 0;  //ADDR_Z
        endcase
    end


    // interrupt handling
    logic nmi_event, nmi_handled, irq_event, rst_event  /*verilator public*/;
    // verilator lint_off SYMRSVDWORD
    wire int_event = nmi_event || irq_event;
    // verilator lint_on SYMRSVDWORD
    always_ff @(posedge clk) begin
        if (rst) begin
            nmi_event   <= 0;
            irq_event   <= 0;
            rst_event   <= 1;
            nmi_handled <= 0;
        end else if(rdy) begin

            nmi_event <= nmi && !nmi_handled;
            if (irq && !p[2]) irq_event <= 1;

            if (brk_int) begin
                nmi_handled <= nmi_event;
                nmi_event   <= 0;
                irq_event   <= 0;
                rst_event   <= 0;
            end

            if (!nmi) nmi_handled <= 0;
        end
    end

    // opcode fetch and interrupt injection
    logic sync_r;
    always_ff @(posedge clk) begin
        if(rdy) begin
            sync_r <= sync; // sync is opcode fetch, sync_r is opcode read
            // update instruction register and inject BRK (op=0) on interrupt
            if (sync_r) ir <= int_event ? '0 : dl; 
        end
        if (rst) begin
            sync_r <= '0;
            ir <= '0;
        end
    end

    // decode instruction
    logic [4:0] op_type;
    logic [2:0] op_src, op_dst;
    logic [8:0] op_alu, alu_code;
    logic wr_op, alu_en, single_byte, idx_XY;
    logic stack_ap, dbNZ, bit_op, sl_op, sr_op;
    logic clc, cli, clv, cld, sec, sei, sed;
    logic [7:0] result_mask;
    decode u_decode (
        .op         (ir),
        .op_type    (op_type),
        .src        (op_src),
        .dst        (op_dst),
        .alu_op     (op_alu),
        .wr_op      (wr_op),
        .alu_en     (alu_en),
        .single_byte(single_byte),
        .idx_XY     (idx_XY),
        .stack_ap   (stack_ap),
        .dbNZ       (dbNZ),
        .bit_op     (bit_op),
        .sl_op      (sl_op),
        .sr_op      (sr_op),
        .clc        (clc),
        .cli        (cli),
        .clv        (clv),
        .cld        (cld),
        .sec        (sec),
        .sei        (sei),
        .sed        (sed),
        .result_mask(result_mask)
    );


    //alu
    logic [7:0] alu_ai, alu_bi, alu_out;
    logic adl_add, adh_add, db_add;
    logic aluC, aluV, aluB, sumC, sumV;
    assign alu_ai = adl_add ? adl : db_add ? 0 : sb;
    assign alu_bi = db;

    alu u_alu (
        .op   (alu_code),
        .ai   (alu_ai),
        .bi   (alu_bi),
        .ci   (p[0]),
        .out  (alu_out),
        .sumC (aluC),
        .sumV (aluV),
        .sumB (aluB)
    );
    // register alu_outputs
    always_ff @(posedge clk) begin
        if(rdy) begin
            add <= alu_out;
            sumC <= aluC;
            sumV <= aluV;
            bpage <= aluB;
        end
        if(rst) begin
            add <= '0;
            sumC <=  0;
            sumV <=  0;
            bpage <= 0;
        end
    end

    // update status register
    logic [7:0] p_update;
    always_comb begin
        p_update = p;

        // update status from alu result
        if (result_rdy) begin
            if (result_mask[7]) p_update[7] = sb[7];
            if (result_mask[6]) p_update[6] = sumV;
            if (result_mask[1]) p_update[1] = sbz;
            if (result_mask[0]) p_update[0] = sumC;
        end

        // there are a few other special cases where status is updated immediately on exec:
        if (exec) begin
            if (clc) p_update[0] = 0;
            if (cli) p_update[2] = 0;
            if (clv) p_update[6] = 0;
            if (cld) p_update[3] = 0;
            if (sec) p_update[0] = 1;
            if (sei) p_update[2] = 1;
            if (sed) p_update[3] = 1;

            if (dbNZ) begin
                p_update[1] = dbz;
                p_update[7] = db[7];
            end
            if (bit_op) p_update[6] = db[6];
            if (sr_op) p_update[0] = sb[0];  // shift-right carry out
        end

        if (so_re) p_update[6] = 1;  //set overflow on re of SO pin
        if (brk_int) p_update[2] = 1;  //set interrupt bit on BRK

        p_update[4] = 0;  //bit 4 doesnt exist but always reports low
        p_update[5] = 1;  //bit 5 doesnt exist but always reports high
    end

    // control state machine
    (* mark_debug = "true" *) logic [2:0] adl_src, adh_src;
    (* mark_debug = "true" *) logic inc_pc, exec, result_rdy, wr_en;
    (* mark_debug = "true" *) logic jump, brk_int;
    (* mark_debug = "true" *) logic hold_alu;
    control u_control (
        .clk        (clk),
        .rst        (rst),
        .rdy        (rdy),
        .op_type    (op_type),
        .op_alu     (op_alu),
        .op_src     (op_src),
        .op_dst     (op_dst),
        .wr_op      (wr_op),
        .alu_en     (alu_en),
        .single_byte(single_byte),
        .stack_ap   (stack_ap),
        .int_event  (int_event),
        .aluC       (sumC),
        .aluN       (add[7]),
        .idx_XY     (idx_XY),
        .bpage      (bpage),
        .take_branch(take_branch),
        .sl_op      (sl_op),
        .sync       (sync),
        .inc_pc     (inc_pc),
        .adl_src    (adl_src),
        .adh_src    (adh_src),
        .jump       (jump),
        .brk_int    (brk_int),
        .adl_add    (adl_add),
        .db_add     (db_add),
        .sb_src     (sb_src),
        .sb_dst     (sb_dst),
        .db_src     (db_src),
        .db_pull    (db_pull),
        .wr_en      (wr_en),
        .exec       (exec),
        .result_rdy (result_rdy),
        .alu        (alu_code)
    );

    // branch logic
    logic take_branch, bpage;
    always_comb begin
        unique case (ir[7:6])
            2'b00: take_branch = p[7] ^ !ir[5];  // BPL, BMI
            2'b01: take_branch = p[6] ^ !ir[5];  // BVC, BVS
            2'b10: take_branch = p[0] ^ !ir[5];  // BCC, BCS
            2'b11: take_branch = p[1] ^ !ir[5];  // BNE, BEQ
        endcase
    end

    //below are not used internally but helpful for debug

    //instruction pointer: pc of current opcode
    (* mark_debug = "true" *) logic [15:0] ip;
    always_ff @(posedge clk) begin
        if (rst) begin
            ip <= RST_VECTOR;
        end else begin
            if (sync && rdy) ip <= pc;
        end
    end

    int cycle  /*verilator public*/;
    always_ff @(posedge clk) begin
        if (rst) cycle <= 0;
        else if (rdy) cycle <= cycle + 1;
    end

    (* mark_debug = "true" *) logic [9:0] Tstate  /*verilator public*/;
    assign Tstate = u_control.Tstate;
    assign jam    = Tstate == 0;  //if Tstate reaches all zeros we have a jam

endmodule

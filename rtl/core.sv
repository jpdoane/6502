`timescale 1ns/1ps
`include "6502_defs.vh"

// address is registered on rising edge of clk
// data must arrive *before* next rising edge
// if using synchronous memory, it should be clocked on @negedge clk

module core6502 #(
    parameter NMI_VECTOR = 16'hfffa,
    parameter RST_VECTOR = 16'hfffc,
    parameter IRQ_VECTOR = 16'hfffe,
    parameter A_RST      = 8'h0,
    parameter X_RST      = 8'h0,
    parameter Y_RST      = 8'h0,
    parameter S_RST      = 8'hff,
    parameter P_RST      = FL_I | FL_U,
    parameter PC_RST      = 16'h0
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
    (* mark_debug = "true" *) logic [7:0] pch, pcl /*verilator public*/;

    // PC
    (* mark_debug = "true" *) logic [15:0] pc /*verilator public*/;
    logic [15:0] pc_next;
    assign pc_next = inc_pc ? pc+1 : pc;
    assign {pch, pcl} = pc;

    // internal buses
    (* mark_debug = "true" *) logic [7:0] sb, db;
    logic [7:0] sb_src, sb_dst;
    logic [5:0] db_src, db_dst;

    // address bus
    (* mark_debug = "true" *) logic [7:0] adl, adh;

    // registers
    always_ff @(posedge clk ) begin
        if (rst) begin
            a <= A_RST;
            x <= X_RST;
            y <= Y_RST;
            s <= S_RST;
            p <= P_RST;
            pc <= PC_RST;

        end else if(rdy) begin
            case(sb_dst)
                REG_A: a <= sb;
                REG_X: x <= sb;
                REG_Y: y <= sb;
                REG_S: s <= sb;
                default: ;
            endcase

            pc <= jump ? {adh, adl} : pc_next;
            p <= p_update;
            case(db_dst)
                DB_A:    a <= db;
                DB_P:    p <= db;
                DB_PCL:  pc[7:0] <= db;
                DB_PCH:  pc[15:8] <= db;
                default: ;
            endcase

            p[4] <= 0;                  //bit 4 doesnt exist but always reports low
            p[5] <= 1;                  //bit 5 doesnt exist but always reports high
        end
    end

    // sb bus (mainly registers and alu output)
    always_comb begin
        case(sb_src)
            REG_A      : sb = a;
            REG_X      : sb = x;
            REG_Y      : sb = y;
            REG_S      : sb = s;
            REG_ADD    : sb = add;
            REG_D      : sb = data_i;
            REG_PCH    : sb = pc_next[15:8];
            default    : sb = 0;
        endcase
    end

    // db bus (mainly memory and pull/push regs)
    always_comb begin
        case(db_src)
            DB_DATA   : db = data_i;
            DB_A      : db = a;
            DB_P      : db = int_event ? p : p | FL_BU; // set break flag on push unless irq
            DB_PCL    : db = pcl;
            DB_PCH    : db = pch;
            DB_SB     : db = sb;
            default   : db = '0;
        endcase
    end

    // address bus
    always_comb begin
        case(adl_src)
            ADDR_PC:    adl = pc_next[7:0];  
            ADDR_DATA:  adl = db; 
            ADDR_ALU:   adl = add; 
            ADDR_INT:   adl = rst_event ? RST_VECTOR[7:0] :
                              nmi_event ? NMI_VECTOR[7:0] :
                              IRQ_VECTOR[7:0];
            ADDR_STACK: adl = s;    
            ADDR_HOLD:  adl = addr[7:0]; 
            default:    adl = 0;    //ADDR_Z
        endcase

        case(adh_src)
            ADDR_PC:    adh = pc_next[15:8];
            ADDR_DATA:  adh = db; 
            ADDR_ALU:   adh = add; 
            ADDR_INT:   adh = rst_event ? RST_VECTOR[15:8] :
                              nmi_event ? NMI_VECTOR[15:8] :
                              IRQ_VECTOR[15:8];
            ADDR_STACK: adh = s;    
            ADDR_HOLD:  adh = addr[15:8]; 
            default:    adh = 0;    //ADDR_Z
        endcase
    end

    // register addr and data_o
    always_ff @(posedge clk ) begin
        addr <= {adh, adl};
        if(!wr_en | rst | rst_event) begin
            rw <= 1;
        end else begin        
            rw <= 0;
            data_o <= db;
        end

        if(rst) begin
            addr <= '0;
            data_o <= '0;
            rw <= 1;
        end
    end
 

    // interrupt handling
    logic nmi_event, nmi_handled, irq_event, rst_event /*verilator public*/;
    // verilator lint_off SYMRSVDWORD
    wire int_event = nmi_event || irq_event;
    // verilator lint_on SYMRSVDWORD
    always_ff @(posedge clk ) begin
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
    always_ff @(posedge clk ) begin
        if (rst || rst_event || (sync && int_event)) ir <= 0;  //break from RESET_VECTOR
        else if (sync && rdy) ir <= db;
    end

    // decode instruction
    logic [4:0] op_type;
    logic [7:0] op_src, op_dst;
    logic [8:0] op_alu, alu_code;
    logic wr_op, alu_en, single_byte, idx_XY;
    logic stack_ap, and_op, bit_op, sl_op, sr_op;
    logic clc, cli, clv, cld, sec, sei, sed;
    logic [7:0] result_mask;
    decode u_decode(
        .op         (ir),
        .op_type        (op_type ),
        .src            (op_src),
        .dst            (op_dst),
        .alu_op         (op_alu),
        .wr_op          (wr_op),
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
    logic [7:0] alu_ai, alu_bi;
    logic adl_add, adh_add, db_add;
    logic sumC, sumV;
    assign alu_ai = adl_add ? adl :
                    db_add ? 0 :
                    sb;
    assign alu_bi = db;

    alu u_alu(
        .clk    (clk),
        .rst    (rst),
        .op     (alu_code),
        .ai     (alu_ai),
        .bi     (alu_bi),
        .ci     (p[0]),
        .out    (add),
        .sumC   (sumC),
        .sumV   (sumV),
        .bpage  (bpage)
    );

    // update p status register
    logic [7:0] p_update;
    wire dbz = (db==0);
    wire sbz = (sb==0);
    always_comb begin
        p_update = p;
        if ( save_alu ) begin
            if(result_mask[7]) p_update[7] = sb[7];
            if(result_mask[6]) p_update[6] = sumV;
            if(result_mask[1]) p_update[1] = sbz;
            if(result_mask[0]) p_update[0] = sumC;
        end

        if (exec) begin
            if (clc) p_update[0] = 0;
            if (cli) p_update[2] = 0;
            if (clv) p_update[6] = 0;
            if (cld) p_update[3] = 0;
            if (sec) p_update[0] = 1;
            if (sei) p_update[2] = 1;
            if (sed) p_update[3] = 1;

            // there are a few other special cases where alu status is
            // not updated with alu result (result_rdy) but
            // directly with alu input (exec):
            if(and_op | bit_op | (db_dst == DB_A) ) begin
                p_update[1] = dbz;
                p_update[7] = db[7];
            end
            if(bit_op) p_update[6] = db[6];
            if(sr_op) p_update[0] = sb[0]; // shift-right carry out

        end

        if (so_re) p_update[6] = 1;       //set overflow on re of SO pin
        if (brk_int) p_update[2] = 1;     //set interrupt bit on BRK

        p_update[4] = 0;                  //bit 4 doesnt exist but always reports low
        p_update[5] = 1;                  //bit 5 doesnt exist but always reports high
    end

    logic so_r, so_re;
    always_ff @(posedge clk ) so_r <= rst ? 0 : so;
    assign so_re = so & ~so_r;

    // control state machine
    (* mark_debug = "true" *) logic [5:0] adl_src, adh_src;
    (* mark_debug = "true" *) logic inc_pc, exec, save_alu, wr_en;
    (* mark_debug = "true" *) logic jump, brk_int;
    (* mark_debug = "true" *) logic hold_alu;    
    control u_control(
        .clk            (clk),
        .rst            (rst),
        .rdy            (rdy),
        .op_type        (op_type),
        .op_alu         (op_alu),
        .op_src         (op_src),
        .op_dst         (op_dst),
        .wr_op          (wr_op),
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
        .inc_pc         (inc_pc),
        .adl_src        (adl_src),
        .adh_src        (adh_src),
        .jump           (jump),
        .brk_int        (brk_int),
        .adl_add        (adl_add),
        .db_add         (db_add),
        .sb_src         (sb_src),
        .sb_dst         (sb_dst),
        .db_src         (db_src),
        .db_dst         (db_dst),
        .wr_en          (wr_en),
        .exec           (exec),
        .save_alu       (save_alu),
        .alu            (alu_code)
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

    //below are not used internally but helpful for debug

    //instruction pointer: pc of current opcode
    (* mark_debug = "true" *)  logic [15:0] ip;
    always_ff @(posedge clk ) begin
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

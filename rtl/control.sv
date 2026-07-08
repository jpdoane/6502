`timescale 1ns / 1ps
`include "6502_defs.vh"

// 6502 control state machine

module control (
    input  logic       clk,
    input  logic       rst,
    input  logic       rdy,
    input  logic [4:0] op_type,
    input  logic [8:0] op_alu,
    input  logic [2:0] op_src,
    input  logic [2:0] op_dst,
    input  logic       wr_op,
    input  logic       alu_en,
    input  logic       single_byte,
    input  logic       stack_ap,
    input  logic       int_event,
    input  logic       aluC,
    input  logic       aluN,
    input  logic       idx_XY,
    input  logic       bpage,
    input  logic       take_branch,
    input  logic       sl_op,
    output logic       sync,
    output logic       inc_pc,
    output logic [2:0] adl_src,
    output logic [2:0] adh_src,
    output logic       jump,
    output logic       brk_int,
    output logic       adl_add,
    output logic       db_add,       // save data bus to alu
    output logic [2:0] sb_src,       // source of data on sb
    output logic [2:0] sb_dst,       // destiation of data on db (reg writes)
    output logic [5:0] db_src,       // source of data on db
    output logic [3:0] db_pull,      // pull data into a, p, pcl, pch
    output logic       wr_en,
    output logic       exec,
    output logic       result_rdy,
    output logic [8:0] alu           // alu operation
);


    //state machine
    logic [9:0] Tstate = T1;
    logic Tlast, toTrmw;
    always_ff @(posedge clk) begin
        if (rst) begin
            Tstate <= T1;
        end else if (rdy) begin
            if (sync) Tstate <= T1;
            else if (Tlast) Tstate <= T0;
            else if (toTrmw) Tstate <= TRMW1;
            else
                // Advance state machine: T0->T1->T2->...
                Tstate <= Tstate << 1;
        end
    end

    wire rmw = wr_op & alu_en;
    logic [2:0] idx;
    logic push_stack, pop_stack, stack_r;
    logic save_alu;

    always_comb begin
        Tlast      = 0;
        toTrmw     = 0;
        adl_src    = ADDR_PC;
        adh_src    = ADDR_PC;
        inc_pc     = 0;
        exec       = 0;
        // dummy_write    = 0;
        jump       = 0;
        sync       = 0;
        brk_int    = 0;
        adl_add    = 0;
        idx        = idx_XY ? SB_X : SB_Y;
        db_add     = 0;

        push_stack = 0;
        pop_stack  = 0;

        // default alu behavior is to store data in alu register
        alu        = OP_SUM;
        sb_src     = SB_Z;

        db_src     = DB_DATA;
        sb_dst     = SB_Z;
        db_pull    = '0;
        wr_en      = 0;

        exec       = 0;


        // In original 6502, there were two clocks m1 and m2, 180 out of phase.
        // In the first half of each cycle (m1 high, m2 low), the address and
        // write data are latched, and in the 2nd half of the cycle (m1 low, m2 high),
        // the read data is latched. Thus for registered memory, the master bus clock is interpreted to be m2.

        // Moving to an fpga implementaion with a single-clock based on m2 effectively
        // splits the T-cycles across different clock cycles, with the combinatorial logic
        // that sets up address and output data on one cycle and the combinatorial logic
        // that depends on input data on the next. The nomenclature for the T-cycles is
        // already confusing, with the 6502 datasheet defining the fetch/sync cycle as T0
        // and visual6502 as T1.

        // this implementaion matches the 6502 datasheet cycle tables for logic that sets
        // up the memory bus, and is delayed by one cycle for logic that depends on memory reads.
        // For example, the opcode address is put onto the bus in T0, but the opcode is read
        // and decoded in T1.

        // this matches visual6502 rather than 6502 datasheet (which has fetch on T0)
        // $display("State: %b", Tstate); 
        unique case (1'b1)
            Tstate[0]: begin  // T0: execute instruction (except writes) and fetch next opcode
                exec   = !wr_op;
                inc_pc = !single_byte;
                sync   = 1;  // emit sync on opcode fetch
            end
            Tstate[1]: begin  // T1: read and decode opcode
                inc_pc = 1;
            end
            Tstate[2]: begin
                unique case (op_type)
                    OP_ZPG: begin
                        adl_src = ADDR_DATA;  // fetch data at {0,ADL}
                        adh_src = ADDR_Z;
                        if (rmw) toTrmw = 1;  // done unless RMW
                        else begin
                            Tlast = 1;
                            exec  = wr_op;
                        end
                    end
                    OP_ZXY, OP_XIN: begin
                        adl_src = ADDR_DATA;  // fetch {0,BAL} (data discarded)
                        adh_src = ADDR_Z;
                        sb_src  = idx;  // compute BAL+index
                    end
                    OP_INY: begin
                        adl_src = ADDR_DATA;  // fetch BAL at {0,IAL}
                        adh_src = ADDR_Z;
                        sb_src  = SB_DATA;
                        alu     = OP_INC;  // IAL++
                    end
                    OP_PUS: begin
                        db_src = stack_ap ? DB_A : DB_P;
                        wr_en = 1;
                        push_stack = 1;
                        exec = 1;
                        Tlast = 1;
                    end
                    OP_PUL, OP_RTI, OP_RTS: begin
                        pop_stack = 1;
                    end
                    OP_BRA: begin
                        inc_pc  = 1;
                        adl_add = 1;  // compute branch addr: PC+2+offset
                        sync    = !take_branch;  // branch not taken
                    end
                    OP_BRK: begin
                        db_src     = DB_PCH;  // push pch
                        wr_en      = 1;
                        push_stack = 1;
                        inc_pc     = !int_event;  // increment pc on BRK but not on IRQ/NMI
                    end

                    // JSR implementation is a bit covoluted, however this is based on visual 6502 behavior
                    // ADL is fetched at T1 but not used until T6, so we need to store is somewhere in meantime
                    // however, we can stash it in the alu since we need to decrement stack pointer
                    // so the stack register temporarily stores ADL while stack pointer is kept on addr bus and alu
                    // this requires adjusting how the stack machinery works somewhat (e.g. w/ push_no_update signal)
                    OP_JSR: begin
                        adl_src = ADDR_STACK;
                        adh_src = ADDR_STACK;  // point addr to stack 
                        sb_src  = SB_DATA;  // read ADL into stack reg...
                        sb_dst  = SB_S;
                        inc_pc  = 1;
                    end
                    OP_IMM, OP_IMP: begin  // effectively the T0 state for 2-cycle insts.
                        inc_pc = !single_byte;
                        if (sl_op || (~alu_en & wr_op)) begin
                            // For non-alu writes (stores), write data on the sb bus
                            // For left shifts, send sb bus to both inputs: X<<1 = X+X
                            db_src = DB_SB;
                        end
                        exec = 1;
                        sync = 1;
                    end
                    OP_AXY: begin
                        sb_src = idx;  // read BAL, compute BAL+X/Y
                        inc_pc = 1;  // fetch BAH at [PC+2]
                    end
                    default: begin
                        inc_pc = 1;  // store [PC+1] in alu, fetch [PC+2]
                    end
                endcase
            end
            Tstate[3]: begin  // T3
                unique case (op_type)
                    OP_ZXY: begin
                        adl_src = ADDR_ALU;  // fetch data at {0,BAL + X/Y}
                        adh_src = ADDR_Z;
                        if (rmw) toTrmw = 1;
                        else begin
                            Tlast = 1;
                            exec  = wr_op;
                        end
                    end
                    OP_XIN: begin
                        adl_src = ADDR_ALU;  // fetch ADL at {0,BAL+X}
                        adh_src = ADDR_Z;
                        sb_src  = SB_ADD;  // compute BAL+X+1
                        alu     = OP_INC;
                    end
                    OP_INY: begin
                        adl_src = ADDR_ALU;  // fetch BAH at {0,IAL+1}
                        adh_src = ADDR_Z;
                        sb_src  = idx;  // compute BAL+Y
                    end
                    OP_ABS: begin
                        adh_src = ADDR_DATA;
                        adl_src = ADDR_ALU;  // fetch data at {ADH,ADL}
                        if (rmw) toTrmw = 1;  // done unless RMW
                        else begin
                            Tlast = 1;
                            exec  = wr_op;
                        end
                    end
                    OP_AXY: begin
                        adh_src = ADDR_DATA;
                        adl_src = ADDR_ALU;  // {BAH, BAL+X/Y}
                        if (aluC) begin
                            sb_src = SB_DATA;
                            alu    = OP_INC;  // increment BAH on carry
                        end else Tlast = !wr_op;
                    end
                    OP_BRA: begin
                        adh_src = ADDR_PC;
                        adl_src = ADDR_ALU;
                        sb_src  = SB_PCH;  // inc or dec adh based on adl + db result
                        alu     = aluN ? OP_DEC : OP_INC;
                        if (!bpage) begin
                            jump = 1;  // jump to {adh, adl + db} if we didnt cross page boundary
                            sync = 1;
                        end
                    end
                    OP_JUM: begin
                        adh_src = ADDR_DATA;
                        adl_src = ADDR_ALU;  // jump to {ADH, ADL}
                        jump    = 1;
                        sync    = 1;
                    end
                    OP_JIN: begin
                        adh_src = ADDR_DATA;
                        adl_src = ADDR_ALU;  // fetch ADL at {IAH, IAL}
                        sb_src  = SB_ADD;
                        alu     = OP_INC;  // IAL++
                    end
                    OP_PUL: begin
                        adh_src = ADDR_STACK;
                        adl_src = ADDR_ALU;  // pull from incremented sp
                        Tlast   = 1;
                    end
                    OP_RTI, OP_RTS: begin
                        pop_stack = 1;
                    end
                    OP_BRK: begin
                        db_src     = DB_PCL;  // push pcl
                        wr_en      = 1;
                        push_stack = 1;
                    end
                    OP_JSR: begin
                        db_src  = DB_PCH;  // push pch
                        wr_en   = 1;
                        adh_src = ADDR_HOLD;
                        adl_src = ADDR_HOLD;  // hold address at stack
                        adl_add = 1;  // dont actually update stack pointer...
                        alu     = OP_DEC;  // instead, "manually" decrement stack address
                    end
                    default: ;
                endcase
            end
            Tstate[4]: begin  // T4
                unique case (op_type)
                    OP_AXY: begin
                        adh_src = ADDR_ALU;
                        adl_src = ADDR_HOLD;  // fetch data at {BAH+C, BAL+X/Y}
                        if (rmw) toTrmw = 1;  // done unless RMW
                        else begin
                            Tlast = 1;
                            exec  = wr_op;
                        end
                    end
                    OP_XIN: begin
                        adh_src = ADDR_Z;
                        adl_src = ADDR_ALU;  // fetch ADH at {0,BAL+X+1}
                    end
                    OP_INY: begin
                        adh_src = ADDR_DATA;
                        adl_src = ADDR_ALU;  // fetch data at {BAH,BAL+Y}
                        if (aluC) begin
                            alu    = OP_INC;  // increment BAH on carry
                            sb_src = SB_DATA;
                        end else Tlast = !wr_op;
                    end
                    OP_BRA: begin
                        adh_src = ADDR_ALU;
                        adl_src = ADDR_HOLD;  // jump to {adh w/ carry, adl + db}
                        jump    = 1;
                        sync    = 1;
                    end
                    OP_JIN: begin
                        adh_src = ADDR_HOLD;
                        adl_src = ADDR_ALU;  // fetch ADH at {IAH, IAL+1}
                    end
                    OP_BRK: begin
                        db_src     = DB_P;  // push p
                        wr_en      = 1;
                        push_stack = 1;
                    end
                    OP_RTI: begin
                        db_pull[PULL_P] = 1;  // read stack into P (1st pull)
                        pop_stack       = 1;
                    end
                    OP_RTS: begin
                        db_pull[PULL_PCL] = 1;  // read stack into PCL (1st pull)
                        adh_src           = ADDR_STACK;  // pull again from incremented sp
                        adl_src           = ADDR_ALU;
                    end
                    OP_JSR: begin  // push PCL "manually" while stack reg is holding ADL:
                        db_src  = DB_PCL;  // 
                        wr_en   = 1;
                        adh_src = ADDR_HOLD;
                        adl_src = ADDR_ALU;  // point to decremented stack
                        adl_add = 1;  // manually decrement stack address again
                        alu     = OP_DEC;
                    end
                    default: ;
                endcase
            end
            Tstate[5]: begin  // T5
                unique case (op_type)
                    OP_XIN: begin
                        adh_src = ADDR_DATA;
                        adl_src = ADDR_ALU;  // fetch data at {ADH, ADL}
                        Tlast   = 1;
                        exec    = wr_op;
                    end
                    OP_INY: begin
                        adh_src = ADDR_ALU;
                        adl_src = ADDR_HOLD;  // fetch data at {ADH, ADL}
                        Tlast   = 1;
                        exec    = wr_op;
                    end
                    OP_JIN: begin
                        adh_src = ADDR_DATA;
                        adl_src = ADDR_ALU;  // jump to {ADH, ADL}
                        jump    = 1;
                        sync    = 1;
                    end
                    OP_BRK: begin
                        adh_src = ADDR_INT;
                        adl_src = ADDR_INT;  // fetch ADL from interrupt vector
                        adl_add = 1;
                        alu     = OP_INC;  // ADDR_INT++
                        brk_int = 1;
                    end
                    OP_JSR: begin
                        adh_src = ADDR_PC;
                        adl_src = ADDR_PC;  // fetch ADH at [PC+2] (maintain stack in alu)
                        sb_src  = SB_ADD;  // hold alu result
                        alu     = OP_NOP;
                    end
                    OP_RTS: begin
                        db_pull[PULL_PCH] = 1;  // read stack into PCH (2nd pull)
                        adh_src           = ADDR_DATA;  // point at {PCH, PCL} (dummy fetch)
                        adl_src           = ADDR_PC;  // PCL was stored in ALU in previous stage
                        Tlast             = 1;  // unlike all other jumps we *dont* skip T0
                                                // so next instruction will be at {PCH, PCL}++
                    end
                    OP_RTI: begin
                        db_pull[PULL_PCL] = 1;  // read stack into PCL (2nd pull)
                        adh_src = ADDR_STACK;  // pull again from incremented sp
                        adl_src = ADDR_ALU;
                        db_add = 1;  // store PCL in alu (bus is still being used for stack)
                    end
                    default: ;
                endcase
            end
            Tstate[6]: begin  // T6
                unique case (op_type)
                    OP_BRK: begin
                        adh_src = ADDR_INT;
                        adl_src = ADDR_ALU;  // fetch ADH from interrupt vector+1
                    end
                    OP_RTI: begin
                        db_pull[PULL_PCH] = 1;  // read stack into ADL (3rd pull)
                        adh_src           = ADDR_DATA;  // jump to {ADH, ADL}
                        adl_src           = ADDR_ALU;  // ADL was stored in ALU in previous stage
                        jump              = 1;  // 
                        sync              = 1;  //
                    end
                    OP_JSR: begin
                        sb_src  = SB_ADD;  // restore stack from alu
                        sb_dst  = SB_S;
                        adh_src = ADDR_DATA;
                        adl_src = ADDR_STACK;  // jump to subroutine
                        jump    = 1;
                        sync    = 1;
                    end
                    default: ;
                endcase
            end
            Tstate[7]: begin  // T7 (BRK only)
                adh_src = ADDR_DATA;
                adl_src = ADDR_ALU;  // jump to {ADH, ADL}
                jump    = 1;
                sync    = 1;
            end
            Tstate[8]: begin  // TRMW1
                adh_src = ADDR_HOLD;
                adl_src = ADDR_HOLD;
                db_src = DB_DATA;
                wr_en = 1;  // dummy writeback
                exec = 1;
            end
            Tstate[9]: begin  // TRMW2
                adh_src = ADDR_HOLD;
                adl_src = ADDR_HOLD;
                Tlast   = 1;
            end
            default: begin
                // JAMMED!
                // $display("6502 jammed at pc=0x%4h", pc);
                // $finish;
            end
        endcase

        if (Tlast & !rmw) exec = wr_op;

        if (exec) begin  // save op results (non alu ops)
            sb_src = op_src;
            if (alu_en) begin
                alu = op_alu;
            end else begin
                sb_dst = op_dst;
                if (op_dst == SB_DATA) begin
                    db_src = DB_SB;  // write to memory
                    wr_en  = 1;
                end
            end

            if (op_type == OP_PUL) begin
                // pull from stack into A or P
                db_pull[PULL_A] = stack_ap;
                db_pull[PULL_P] = ~stack_ap;
            end
        end

        // save results of alu (delayed due to latency of alu)
        if (save_alu) begin
            sb_src = SB_ADD;
            sb_dst = op_dst;

            // write alu result to mem
            if (op_dst == SB_DATA) begin
                db_src = DB_SB;
                wr_en  = 1;
            end
        end

        // stack sequencing
        if (pop_stack | push_stack) begin
            adh_src = ADDR_STACK;
            adl_src = stack_r ? ADDR_ALU : ADDR_STACK;
            adl_add = 1;
            alu = pop_stack ? OP_INC : OP_DEC;
        end
        // update sp after inc/dec
        if (stack_r) begin
            sb_src = SB_ADD;
            sb_dst = SB_S;
        end

    end

    always_ff @(posedge clk) begin
        if (rst) begin
            stack_r  <= 0;
            save_alu <= 0;
        end else begin
            stack_r  <= pop_stack | push_stack;
            save_alu <= exec & alu_en;
        end
    end

    assign result_rdy = (exec & ~alu_en) | save_alu;


endmodule

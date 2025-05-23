`timescale 1ns/1ps
`include "6502_defs.vh"

// 6502 control state machine

module control (
    input logic clk,
    input logic rst,
    input logic rdy,
    input logic [4:0] op_type,
    input logic wr_op,
    input logic alu_en,
    input logic single_byte,
    input logic stack_ap,
    input logic interrupt,
    input logic aluC,
    input logic aluN,
    input logic idx_XY,
    input logic bpage,
    input logic take_branch,
    input logic sl_op,

    output logic sync,
    output logic exec,
    output logic [3:0] alu_flags,
    output logic [6:0] sb_src,
    output logic inc_pc,
    output logic [5:0] adl_src,
    output logic [5:0] adh_src,
    output logic write_mem,
    output logic dummy_write,
    output logic jump,
    output logic brk_int,
    output logic adl_add,
    output logic alu_az,
    output logic [3:0] stack_push,
    output logic [3:0] stack_read,
    output logic sb_s,
    output logic sb_db
    );

    //state machine
    logic [9:0] Tstate=0;
    logic Tlast, skipT0, toTrmw;
    always @(posedge clk ) begin
        if (rst) begin
            Tstate <= T1;
        end
        else if (rdy) begin
            if (skipT0)
                Tstate <= T1;
            else if (Tlast)
                Tstate <= T0;
            else if (toTrmw)
                Tstate <= TRMW1;
            else
                Tstate <= Tstate << 1;
        end
    end

    wire rmw = wr_op & alu_en;
    logic push_no_update;
    logic [6:0] idx;
    always_comb begin
        Tlast       = 0;
        skipT0      = 0;
        toTrmw      = 0;
        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 0;
        exec        = 0;
        dummy_write = 0;
        jump        = 0;
        sync        = 0;
        brk_int     = 0;
        adl_add     = 0;
        idx         = idx_XY ? REG_X : REG_Y;
        stack_push  = 0;
        stack_pull  = 0;
        push        = 0;
        pull        = 0;
        push_no_update = 0;
        sb_s        = 0;
        sb_db        = 0;
        alu_az      = 0;

        // default alu behavior is to store data in alu register
        alu_flags = ALU_NOF;
        sb_src = REG_Z;

        // relative to visual6502/perfect6502, the address bus is output one clock early
        // relative to 6502 datasheet for given Tstate, address bus matches, but data/execution is one state late
        // e.g. opcode is fetched on T0 and received on data bus on T1
        unique case (1'b1)
            Tstate[0]: begin                                            // T0
                exec = !wr_op;                                          // execute previous instruction (except writes)
                inc_pc = !single_byte;                                  // fetch opcode [PC]
                end
            Tstate[1]: begin                                            // T1
                sync = 1;                                               // emit sync on opcode read
                inc_pc = 1;                                             // fetch [PC+1]
                end
            Tstate[2]: begin                                            // T2
                unique case(op_type)
                    OP_ZPG: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};       // fetch data at {0,ADL}
                        if(rmw) toTrmw = 1;                             // done unless RMW
                        else Tlast = 1;
                    end
                    OP_ZXY, OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};       // fetch {0,BAL} (data discarded)
                        sb_src = idx;                                   // compute BAL+index
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};       // fetch BAL at {0,IAL}
                        sb_src = REG_D;
                        alu_flags = ALU_INC;                                // IAL++
                    end
                    OP_PUS: begin
                        stack_push = stack_ap ? STACK_A : STACK_P;      // push a or p
                        Tlast = 1;
                    end
                    OP_PUL: begin
                        stack_pull = stack_ap ? STACK_A : STACK_P;      // pull a or p
                    end
                    OP_BRA: begin       
                        inc_pc = 1;            
                        adl_add = 1;                                    // compute branch addr: PC+2+offset
                        skipT0 = !take_branch;                            // branch not taken
                    end
                    OP_BRK: begin
                        stack_push = STACK_PCH;                         // push pch
                        inc_pc = !interrupt;                            // increment pc on BRK but not on IRQ/NMI
                    end

                    // JSR implementation is a bit covoluted, however this is based on visual 6502 behavior
                    // ADL is fetched at T1 but not used until T6, so we need to store is somewhere in meantime
                    // however, we can stash it in the alu since we need to decrement stack pointer
                    // so the stack register temporarily stores ADL while stack pointer is kept on addr bus and alu
                    // this requires adjusting how the stack machinery works somewhat (e.g. w/ push_no_update signal)
                    OP_JSR: begin
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};  // point addr to stack 
                        sb_src = REG_D;                                 // read ADL into stack reg...
                        sb_s = 1;                                       
                        inc_pc = 1;                                     // pc++
                    end
                    OP_RTS: stack_pull = STACK_PCL;
                    OP_RTI: stack_pull = STACK_P;
                    OP_IMM,
                    OP_IMP: begin                                       // effectively the T0 state for 2-cycle insts.
                        inc_pc = !single_byte;
                        sb_db = sl_op;                                  // db <= sb on left shifts
                        exec = 1;
                        skipT0 = 1;
                    end
                    OP_AXY: begin
                        sb_src = idx;                                   // read BAL, compute BAL+X/Y
                        inc_pc = 1;                                     // fetch BAH at [PC+2]
                    end
                    default: inc_pc = 1;                                // store [PC+1] in alu, fetch [PC+2]
                endcase
                end
            Tstate[3]: begin                                            // T3
                unique case(op_type)
                    OP_ZXY: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};        // fetch data at {0,BAL + X/Y}
                        if(rmw) toTrmw = 1;
                        else Tlast = 1;
                    end
                    OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};        // fetch ADL at {0,BAL+X}
                        sb_src = REG_ADD;                               // compute BAL+X+1
                        alu_flags = ALU_INC;
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};        // fetch BAH at {0,IAL+1}
                        sb_src = idx;                                   // compute BAL+Y
                    end
                    OP_ABS: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // fetch data at {ADH,ADL}
                        if(rmw) toTrmw = 1;                             // done unless RMW
                        else Tlast = 1;
                    end
                    OP_AXY: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // {BAH, BAL+X/Y}
                        if (aluC) begin
                            sb_src = REG_D;
                            alu_flags = ALU_INC;            // increment BAH on carry
                        end
                        else Tlast = !wr_op;
                    end
                    OP_BRA: begin                        
                        {adh_src, adl_src} = {ADDR_PC, ADDR_ALU};
                        sb_src = REG_ADH;                                    // inc or dec adh based on adl + db result
                        alu_flags = aluN ? ALU_DEC : ALU_INC;
                        if (!bpage) begin
                            jump = 1;                                  // jump to {adh, adl + db} if we didnt cross page boundary
                            skipT0 = 1;                                
                        end
                    end
                    OP_JUM: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // jump to {ADH, ADL}
                        jump = 1;
                        skipT0 = 1;                                
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // fetch ADL at {IAH, IAL}
                        sb_src = REG_ADD;
                        alu_flags = ALU_INC;                               // IAL++
                    end
                    OP_PUL: begin
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
                        Tlast = 1;
                    end
                    OP_RTS: stack_pull = STACK_PCH;
                    OP_BRK: stack_push = STACK_PCL;
                    OP_RTI: stack_pull = STACK_PCL;

                    OP_JSR: begin                                       
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_HOLD};    // hold address at stack
                        stack_push = STACK_PCH;                         // write pch to stack
                        push_no_update = 1;                             // but dont actually update stack pointer...
                        adl_add = 1;                                    // instead, "manually" decrement stack address
                        alu_flags = ALU_DEC;
                    end
                    default: ;
                endcase
                end
            Tstate[4]: begin                                            // T4
                unique case(op_type)
                    OP_AXY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};     // fetch data at {BAH+C, BAL+X/Y}
                        if(rmw) toTrmw = 1;                             // done unless RMW
                        else Tlast = 1;
                    end
                    OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};        // fetch ADH at {0,BAL+X+1}
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // fetch data at {BAH,BAL+Y}
                        if (aluC) begin
                            alu_flags = ALU_INC;                     // increment BAH on carry
                            sb_src = REG_D;
                        end
                        else Tlast = !wr_op;
                    end
                    OP_BRA: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};     // jump to {adh w/ carry, adl + db}
                        jump = 1;
                        skipT0 = 1;                                
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_ALU};     // fetch ADH at {IAH, IAL+1}
                    end
                    OP_BRK: stack_push = STACK_P;
                    OP_RTI: stack_pull = STACK_PCH;
                    OP_JSR: begin                                       // push PCL "manually" while stack reg is holding ADL:
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_ALU};     // point to decremented stack
                        stack_push = STACK_PCL;                         // write pcl to stack
                        push_no_update = 1;                             // dont update stack register...
                        adl_add = 1;                                    // manually decrement stack address again
                        alu_flags = ALU_DEC;
                    end
                    OP_RTS: begin
                        alu_az = 1;                                     // read PCL into alu (while sb used for stack)
                    end
                    default: ;
                endcase
                end
            Tstate[5]: begin                                            // T5
                unique case(op_type)
                    OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // fetch data at {ADH, ADL}
                        Tlast = 1;
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};     // fetch data at {ADH, ADL}
                        Tlast = 1;
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // jump to {ADH, ADL}
                        jump = 1;
                        skipT0 = 1;                                
                    end
                    OP_BRK: begin
                        {adh_src, adl_src} = {ADDR_INT, ADDR_INT};      // fetch ADL from interrupt vector
                        adl_add = 1;
                        alu_flags = ALU_INC;                               // ADDR_INT++
                        brk_int = 1;
                    end
                    OP_JSR: begin                                       // fetch ADH at [PC+2]
                        sb_src = REG_ADD;                               // maintain stack in alu
                        alu_flags = ALU_BIZ;                               
                    end
                    OP_RTS: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // point at {PCH, PCL} (dummy fetch)
                        jump = 1;
                        Tlast = 1;                                      // unlike all other jumps we *dont* skip T0
                                                                        // so next instruction will be at {PCH, PCL}++
                    end
                    OP_RTI: begin
                        alu_az = 1;                                     // read ADH into alu
                    end
                    default: ;
                endcase
                end                
            Tstate[6]: begin                                            // T6
                unique case(op_type)
                    OP_BRK: begin
                        {adh_src, adl_src} = {ADDR_INT, ADDR_ALU};      // fetch ADH from interrupt vector+1
                    end
                    OP_RTI: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // jump to {ADH, ADL}
                        jump = 1;
                        skipT0 = 1;                                
                    end
                    OP_JSR: begin
                        sb_src = REG_ADD;                               // restore stack reg from alu
                        sb_s = 1;
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_STACK};   // jump to {ADH, ADL}
                        jump = 1;
                        skipT0 = 1;                                
                    end
                    default: ;
                endcase
                end      
            Tstate[7]: begin                                            // T7 (BRK only)
                    {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};         // jump to {ADH, ADL}
                    jump = 1;
                    skipT0 = 1;                                
                end      
            Tstate[8]: begin                                            // TRMW1
                    {adh_src, adl_src} = {ADDR_HOLD, ADDR_HOLD};
                    dummy_write = 1;
                    exec = 1;
                    end
            Tstate[9]: begin                                            // TRMW2
                    {adh_src, adl_src} = {ADDR_HOLD, ADDR_HOLD};
                    Tlast = 1;
                    end

            default: begin
                // $display("6502 jammed at pc=0x%4h", pc);
                // $finish;
                end                
        endcase

        // stack logic
        push = (stack_push != 0) & !push_no_update;
        pull = stack_pull != 0;
        if (push | pull) begin    // calc new stack ptr
            alu_flags = push ? ALU_DEC : ALU_INC;
            adh_src = ADDR_STACK;
            sb_src = REG_S;
            adl_src = stack_r ? ADDR_ALU : ADDR_STACK;
        end
        if (stack_r) begin  // update stack ptr
            sb_src = REG_ADD;
            sb_s = 1;
        end

        // pull read occurs on following cycle, after incrementing stack address
        if (pull_r)
            {adh_src, adl_src} = {ADDR_STACK, ADDR_ALU};

        write_mem = ( wr_op & Tlast ) | push_no_update | push | dummy_write;
    end


    // stack
    logic push, pull, pull_r, stack_r;
    logic [3:0] stack_pull, stack_pull_r;
    always @(posedge clk ) begin
        if (rst) begin
            pull_r <= 0;
            stack_r <= 0;
            stack_pull_r <= 0;
            stack_read <= 0;
        end else begin
            pull_r <= pull;
            stack_r <= push | pull;
            stack_pull_r <= stack_pull;
            stack_read <= stack_pull_r;
        end
    end


endmodule

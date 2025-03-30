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
    input logic [7:0] alu_status,
    input logic [5:0] alu_op_exec,
    input logic [5:0] sb_src_exec,
    input logic idx_XY,
    input logic bpage,

    output logic [7:0] Tstate,
    output logic sync,
    output logic exec,
    output logic [5:0] alu_op,
    output logic [5:0] sb_src,
    output logic inc_pc,
    output logic [2:0] adl_src,
    output logic [2:0] adh_src,
    output logic write_mem,
    output logic dummy_write,
    output logic jump,
    output logic handle_int,
    output logic adl_add,
    output logic adh_add,
    output logic [3:0] stack_push,
    output logic [3:0] stack_read,
    output logic up_s,
    output logic jam
    );

    //state machine
    logic Tlast, skipT0, toTrmw;
    always @(posedge clk ) begin
        if (rst) begin
            Tstate <= T1;
        end
        else if (rdy) begin

            Tstate <=   Tlast ? skipT0 ? T1 : T0 :
                        toTrmw ? TRMW1 : 
                        Tstate >> 1;

        end
    end

    assign jam = !|Tstate; //if Tstate reaches all zeros we have a jam

    wire rmw = wr_op & alu_en;
    logic push_no_update;
    logic [5:0] idx;
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
        handle_int  = 0;
        adl_add     = 0;
        adh_add     = 0;
        idx         = idx_XY ? REG_X : REG_Y;
        stack_push  = 0;
        stack_pull  = 0;
        push        = 0;
        pull        = 0;
        push_no_update = 0;
        up_s        = 0;

        // default alu behavior is to store data in alu register
        alu_op = ALU_SUM;
        sb_src = REG_Z;

        // relative to visual6502/perfect6502, we output address one clock early
        // relative to 6502 datasheet, address bus matches for given T state, and data/execution is one state late (opcode is read on T1, not T0)
        case (Tstate)
            T0: begin
                exec = !wr_op;                                          // execute instruction
                inc_pc = !single_byte;                                  // fetch next opcode
                end
            T1: begin
                sync = 1;                                               // emit sync on opcode read
                inc_pc = 1;                                             // fetch [PC+1]
                end
            T2: begin
                case(op_type)
                    OP_ZPG: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};       // fetch data from {0,ADL}
                        if(rmw) toTrmw = 1;
                        else Tlast = 1;
                    end
                    OP_ZXY, OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};       // fetch {0,BAL} (discarded)
                        sb_src = idx;                                  // BAL+index
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_DATA};       // fetch BAL from {0,IAL}
                        alu_op = ALU_INB;                               // IAL++
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
                        adl_add = 1;                                    // adl + db
                    end
                    OP_BRK: begin
                        stack_push = STACK_PCH;                         // push pch
                        inc_pc = !interrupt;                            // pc++ on BRK, not on IRQ/NMI
                    end
                    OP_JSR: begin
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};  // point addr to stack 
                                                                        // read ADL and store in alu...
                        inc_pc = 1;                                     // pc++
                    end
                    OP_RTS: begin
                        stack_pull = STACK_PCL;
                    end
                    OP_RTI: begin
                        stack_pull = STACK_P;
                    end
                    OP_IMM,
                    OP_IMP,
                    OP_BNT: begin                                       // effectively the T0 state for 2-cycle insts.
                        inc_pc = !single_byte;
                        exec = 1;
                        skipT0 = 1;
                        Tlast = 1;
                    end
                    OP_AXY: begin
                        sb_src = idx;                                   // read BAL, compute BAL+X/Y
                        inc_pc = 1;                                     // fetch BAH from PC+2
                    end
                    default: begin  // other ops: OP_ABS, OP_JMP
                        inc_pc = 1;                                     // store [PC+1] in alu, fetch [PC+2]
                    end
                endcase
                end
            T3: begin
                case(op_type)
                    OP_ZXY: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};        // fetch data from {0,BAL + X/Y}
                        if(rmw) toTrmw = 1;
                        else Tlast = 1;
                    end
                    OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};        // fetch ADL from {0,BAL+X}
                        sb_src = REG_ADD;                                  // compute BAL+X+1
                        alu_op = ALU_INC;
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};        // fetch BAH from {0,IAL+1}
                        sb_src = idx;                                      // compute BAL+Y
                    end
                    OP_ABS: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // fetch data from {ADH,ADL}
                        if(rmw) toTrmw = 1;
                        else Tlast = 1;
                    end
                    OP_AXY: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // {BAH, BAL+X/Y}
                        if (alu_status[0]) alu_op = ALU_INB;            // increment BAH on carry
                        else Tlast = !wr_op;
                    end
                    OP_BRA: begin                        
                        {adh_src, adl_src} = {ADDR_PC, ADDR_ALU};
                        jump = !bpage;                                  // jump to {adh, adl + db} if we didnt cross page boundary
                        adh_add = 1;                                    // inc or dec adh based on adl + db result
                        alu_op = alu_status[7] ? ALU_DEC : ALU_INC;              
                    end
                    OP_JUM: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        jump = 1;
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // fetch ADL from {IAH, IAL}
                        sb_src = REG_ADD;
                        alu_op = ALU_INC;                               // IAL++
                    end
                    OP_PUL: begin
                        {adh_src, adl_src} = {ADDR_STACK, ADDR_STACK};
                        Tlast = 1;
                    end
                    OP_BRK: begin
                        stack_push = STACK_PCL;
                    end
                    OP_JSR: begin
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_HOLD};    // hold addr at stack 
                        stack_push = STACK_PCH;                         // write pch to stack
                        push_no_update = 1;                             // but dont actually update stack pointer...
                        up_s = 1;                                       // instead store alu in s (via alu)
                        adl_add = 1;                                    // and manually decrement stack address in alu
                        alu_op = ALU_DEC;
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
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};  // fetch data from {BAH+C, BAL+X/Y}
                        if(rmw) toTrmw = 1;
                        else Tlast = 1;
                    end
                    OP_XIN: begin
                        {adh_src, adl_src} = {ADDR_Z, ADDR_ALU};    // fetch ADH from {0,BAL+X+1}
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU}; // fetch data from {BAH,BAL+Y}
                        if (alu_status[0]) alu_op = ALU_INB;        // increment BAH on carry
                        else Tlast = !wr_op;
                    end
                    OP_BRA: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};   // jump to {adh w/ carry, adl + db}
                        jump = 1;
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_ALU}; // fetch ADH from {IAH, IAL+1}
                    end
                    OP_BRK: begin
                        stack_push = STACK_P;
                    end
                    OP_JSR: begin
                        {adh_src, adl_src} = {ADDR_HOLD, ADDR_ALU}; // decremented stack
                        stack_push = STACK_PCL;                     // write pcl to stack
                        push_no_update = 1;                         // but dont actually update stack register...
                        adl_add = 1;                                // manually decrement stack address
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
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // fetch data from {ADH, ADL}
                        Tlast = 1;
                    end
                    OP_INY: begin
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_HOLD};     // fetch data from {ADH, ADL}
                        Tlast = 1;
                    end
                    OP_JIN: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // jump to {ADH, ADL}
                        jump = 1;
                    end
                    OP_BRK: begin
                        {adh_src, adl_src} = {ADDR_INT, ADDR_INT};      // fetch ADL from interrupt vector
                        adl_add = 1;
                        alu_op = ALU_INC;                               // ADDR_INT++
                    end
                    OP_JSR: begin                                       // fetch ADH from PC+2
                        sb_src = REG_S;                                    // move ADL from stack register to alu
                        alu_op = ALU_NOP;
                        up_s = 1;                                       // restore stack register from alu
                    end
                    OP_RTS: begin
                        jump = 1;
                    end
                    default: begin end
                endcase
                end                
            T6: begin
                case(op_type)
                    OP_BRK: begin
                        {adh_src, adl_src} = {ADDR_INT, ADDR_ALU};      // fetch ADH from interrupt vector+1
                    end
                    OP_JSR: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // read in ADH, and restore ADL from alu
                        jump = 1;                                       // jump to {ADH, ADL}
                    end
                    OP_RTI: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};
                        jump = 1;
                    end
                    default: begin end
                endcase
                end      
            T7: begin
                case(op_type)
                    OP_BRK: begin
                        {adh_src, adl_src} = {ADDR_DATA, ADDR_ALU};     // jump to {ADH, ADL}
                        handle_int = 1;
                        jump = 1;
                    end
                    default: begin end
                endcase
                end      

            TRMW1: begin
                    {adh_src, adl_src} = {ADDR_HOLD, ADDR_HOLD};
                    dummy_write = 1;
                    exec = 1;
                    end
            TRMW2: begin
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
        if (push | pull) begin
            // point at stack, and either increment or decrment
            adh_src = ADDR_STACK;
            // if we just did a stack op, get stack from alu
            adl_src = (push_r | pull_r) ? ADDR_ALU : ADDR_STACK;
            sb_src = (push_r | pull_r) ? REG_ADD : REG_S;
            alu_op = push ? ALU_DEC : ALU_INC;
        end
        if (pull_r) begin // pull occurs on incremented address
            {adh_src, adl_src} = {ADDR_STACK, ADDR_ALU};
        end
        if (pull_r | push_r) up_s = 1;  // update the stack ptr

        // config ALU to execute opcode
        if (exec) begin
            alu_op = alu_op_exec;
            sb_src = sb_src_exec;
        end

        if (jump) begin
            skipT0 = 1;
            Tlast = 1;
        end
    end

    assign write_mem = push_no_update | push | dummy_write | ( wr_op & Tlast );

    // stack
    logic push, pull, push_r, pull_r;
    logic [3:0] stack_push_r, stack_pull, stack_pull_r;
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


endmodule

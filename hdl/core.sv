`timescale 1ns/1ps
`include "6502_defs.vh"

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

    // because the FPGA implementation uses synchronous memory,
    // the output address bus appears one clock earlier than in a real 6502
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

   
    // ready signal
    wire rdy = ready | ~rw; //ignore not ready when writing

    // registers
    logic [7:0] ir /*verilator public*/;
    logic [7:0] add /*verilator public*/;
    logic [7:0] a /*verilator public*/;
    logic [7:0] s /*verilator public*/;
    logic [7:0] x /*verilator public*/;
    logic [7:0] y /*verilator public*/;
    logic [7:0] p /*verilator public*/;

    // wire [7:0] s_passthrough = up_s ? add : s;  // stack pointer with immediate update

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
    logic [7:0] adl_r,adh_r;
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
            default:    adl = pcl; //ADDR_PC
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
            default:    adh = pch; //ADDR_PC
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
            default       : sb_result = 0;
        endcase
        
        // data write bus
        case(1'b1)
            stack_push[0]:  db_result = a;
            stack_push[1]:  db_result = interrupt ? p : p | FL_BU; // set break flags unless irq
            stack_push[2]:  db_result = pcl;
            stack_push[3]:  db_result = pch;
            dummy_write:    db_result = db;
            default         db_result = sb_result;
        endcase
    end

    // i/o data registers
    assign rw = !write_mem | rst | rst_event;
    logic [7:0] data_r;
    always_ff @(posedge clk) begin
        if (rst) begin
            data_r <= 0;
        end else begin
            data_r <= data_i;
        end
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

    logic fetch_intr;
    wire IRQ_masked = irq && !p[2];
    always @(posedge clk ) begin
        if (rst) begin
            nmi_event <= 0;
            irq_event <= 0;
            rst_event <= 1;
            nmi_handled <= 0;
            // fetch_intr <= 0;
        end else begin

            nmi_event <= nmi && !nmi_handled;
            if (IRQ_masked)
                irq_event <= 1;
            
            if(handle_int) begin
                nmi_handled <= nmi_event;
                nmi_event <= 0;
                irq_event <= 0;
                rst_event <= 0;
            end

            if(!nmi) nmi_handled <= 0;

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
    logic [5:0] sb_src, sb_src_exec, dst;
    logic alu_en;
    logic [5:0] alu_op, alu_op_exec;
    logic upNZ, upV, upC;
    logic wr_op, single_byte, idx_XY;
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
    assign wr_op = dst[5];

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
    logic [7:0] alu_ai, alu_bi, alu_status;
    logic adl_add, adh_add;

    // in real 6502, adl is routed to alu_bi and adh is routed to alu_ai via sb, controlled at subcycle
    // we are also cheating here a bit by using the registered addr than adl/adh bus, but this accomplishes the same effect
    assign  alu_ai = adl_add ? adl :
                     adh_add ? adh :
                     sb;
    assign alu_bi = db; 
    alu u_alu(
        .clk    (clk),
        .rst    (rst),
        .op     (alu_op  ),
        .ai     (alu_ai  ),
        .bi     (alu_bi  ),
        .ci     (p[0]),
        .out    (add),
        .status (alu_status)
    );

    // always @(posedge clk ) begin
    //     if (rst) add <= 0;
    //     else if (!hold_alu) add <= alu_out;
    // end

    //update registers
    logic so_r;
    logic exec_r;
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

    // control
    logic bpage, bpage_up, bpage_down;
    assign bpage_up = pc[7] & !data_r[7] & !alu_status[7]; // crossed to next page if base>127, offset>0, and result <= 127
    assign bpage_down = !pc[7] & data_r[7] & alu_status[7]; // crossed to prev page if base<=127, offset<0, and result > 127
    assign bpage = bpage_up || bpage_down;

    wire rmw = wr_op & alu_en;
    logic exec;
    logic push_no_update, up_s;
    logic [5:0] idx;
    always_comb begin
        Tlast       = 0;
        skipT0      = 0;
        toTrmw      = 0;
        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 0;
        hold_alu    = 0;
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
                        {adh_src, adl_src} = {ADDR_ALU, ADDR_PC};   // jump to {adh w/ carry, adl + db}
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

    assign jam = !|Tstate; //if Tstate reaches all zeros we have a jam

    //instruction pointer: pc of current opcode
    //unused by core, but helpful for debug
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

    // logic [15:0] addr;
    // logic [7:0] dor;
    // logic RW
    // always @(posedge clk ) begin
    //     addr <= addr;
    //     dor <= data_o;
    //      RW <= rw;
    // end


endmodule

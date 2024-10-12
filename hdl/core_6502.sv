`timescale 1ns/1ps
`include "6502_defs.vh"

module core_6502 #(
    parameter NMI_VECTOR=16'hfffa,
    parameter RST_VECTOR=16'hfffc,
    parameter IRQ_VECTOR=16'hfffe,
    parameter INITIAL_STATUS=FL_I | FL_U
    )
    (
    input  logic i_clk, i_rst,
    input  logic [7:0] i_data,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,

    output logic [15:0] addr,
    output logic [7:0] dor,
    output logic RW,
    output logic sync,
    output logic jam

    `ifdef DEBUG_REG
        ,input  logic reg_set_en,
        input  logic [7:0] pc_set,
        input  logic [7:0] s_set,
        input  logic [7:0] a_set,
        input  logic [7:0] x_set,
        input  logic [7:0] y_set,
        input  logic [7:0] p_set
    `endif 

    );

`ifdef DEBUG_CPU
    `include "debug/debug.sv"
`endif 

    // busses
    // db: data bus: typically memory read, but can be addr_lo or zero
    // sb: secondry bus: read from registers (a,x,y,s, zero)
    logic [7:0] db, sb;

    // ready signal
    wire rdy = READY | ~RW; //ignore not ready when writing
    logic rdy_r;
    logic [7:0] data, data_reg;
    always_ff @(posedge i_clk) begin
        if (i_rst) begin
            rdy_r <= 0;
            data_reg <= 0;
        end else begin
            rdy_r <= rdy;
            data_reg <= data;
        end
    end

    // when not ready, use registered data (i.e. remember old data)
    // otherwise, passthrough current data
    assign data = rdy_r ? i_data : data_reg;

    // address registers
    logic [7:0] adl,adh;
    logic [7:0] pcl, pch;
    logic [15:0] pc, pcsel, nextpc;

    // registers
    logic [7:0] ir, add;
    logic [7:0] a, s, x, y, p;

      // state
    logic [5:0] state, initial_state;

    //alu
    logic [2:0] alu_OP;
    logic [7:0] ai, bi;
    logic [7:0] alu_out;
    logic ci, aluN, aluV, aluZ, aluC;
    logic ai_inv, bi_inv, adl_bi;

    // decode 
    logic [7:0] opcode;
    logic single_byte;
    logic idx_XY;           // select index X(1) or Y(0) for abs,X/Y and zpg,X/Y ops
    logic mem_read, mem_write, reg_write;
    logic alu_en;
    logic [2:0] op_alu_OP;
    logic op_ai_inv;   // alu ai src
    logic op_bi_inv;   // alu bi src
    logic op_Pci, op_ci;       // carry in = ci || (Pci & p[0])
    logic [2:0] op_db_src,op_sb_src,op_dst;
    logic setV, setC;    // update p C or V bits with alu result

   // control signals
    logic [2:0] db_src,sb_src,dst;
    logic [2:0] adl_src,adh_src;
    logic store, load, rmw;
    logic ipc;
    logic inc_pc;
    logic push, pop, rpop;
    logic skip;
    logic exec, rexec;
    logic save;
    logic jump, handle_int;
    logic holdalu;
    logic db_write;
    logic sb_a;
    logic sb_x;
    logic sb_y;
    logic sb_s;
    logic db_p;
    logic[7:0] set_mask, clear_mask;
    logic ipage_up, bpage_up, bpage_down;

    //address and pc logic
    always @(*) begin
        case(adl_src)
            ADDR_PC: adl = pc[7:0];
            ADDR_DATA: adl = data;
            ADDR_ADD: adl = add;
            ADDR_Z: adl = 8'b0;
            ADDR_INT: adl = rst_event ? RST_VECTOR[7:0] :
                            nmi_event ? NMI_VECTOR[7:0] :
                            IRQ_VECTOR[7:0];
            ADDR_STACK: adl = s;
            default: adl = pcl; //ADDR_HOLD
        endcase
        case(adh_src)
            ADDR_PC: adh = pc[15:8];
            ADDR_DATA: adh = data;
            ADDR_ADD: adh = add;
            ADDR_Z: adh = 8'b0;
            ADDR_INT: adh = rst_event ? RST_VECTOR[15:8] :
                            nmi_event ? NMI_VECTOR[15:8] :
                            IRQ_VECTOR[15:8];
            ADDR_STACK: adh = STACKPAGE;
            default: adh = pch; //ADDR_HOLD
        endcase
    end

    logic nmi_event, nmi_handled, irq_event, rst_event;
    logic fetch_intr;
    wire IRQ_masked = IRQ && !p[2];
    always @(posedge i_clk ) begin
        if (i_rst) begin
            nmi_event <= 0;
            irq_event <= 0;
            rst_event <= 1;
            nmi_handled <= 0;
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

            // set ir to BRK (0) rather than fetched instruction
            fetch_intr <= nmi_event | irq_event | rst_event;
        end
    end

    // opcode fetch and interrupt injection
    wire [7:0]  op_fetch = fetch_intr ? 0 : data;
    assign opcode = (state==T1_DECODE) ? op_fetch : ir;
    always @(posedge i_clk ) begin
        if (i_rst) ir <= 0;  //break from RESET_VECTOR
        else if (rdy) ir <= opcode;
    end

    //instruction pointer: pc of current opcode
    //unused by core, but helpful for sim/debug
// `ifndef SYNTHESIS
    (* mark_debug = "true" *)  logic [15:0] ip;
    always @(posedge i_clk ) begin
        if (i_rst) ip <= RST_VECTOR;
        else if (sync && rdy) ip <= addr;
    end
// `endif 

    assign addr = {adh, adl};
    assign pcsel = jump ? addr : pc;
    assign nextpc = (sync | inc_pc | jump) && !((nmi_event || irq_event) && sync) ? pcsel + 1 : pcsel;
    // assign nextpc = (sync | inc_pc | jump) ? pcsel + 1 : pcsel;


    // decode instruction
    decode u_decode(
        .i_clk         (i_clk         ),
        .i_rst         (i_rst         ),
        .rdy           (rdy),
        .opcode        (opcode        ),
        .pstatus       (p             ),
        .initial_state (initial_state ),
        .single_byte   (single_byte   ),
        .idx_XY        (idx_XY   ),
        .mem_read       (mem_read),
        .mem_write      (mem_write),
        .reg_write      (reg_write),
        .sb_src        (op_sb_src        ),
        .db_src        (op_db_src        ),
        .dst       (op_dst       ),
        .alu_en        (alu_en        ),
        .alu_OP        (op_alu_OP        ),
        .ai_inv        (op_ai_inv        ),
        .bi_inv        (op_bi_inv        ),
        .Pci            (op_Pci      ),
        .ci             (op_ci      ),
        .setV             (setV      ),
        .setC             (setC      ),
        .set_mask      (set_mask      ),
        .clear_mask    (clear_mask    )
    );
    assign rmw = mem_read && mem_write;        //ALU OP, read/save to/from mem
    assign store = !alu_en && mem_write;       //ALU NOP, save to mem (includes push)
    assign load = !alu_en && reg_write;        //ALU NOP, save to reg (includes pull, transfer)


    // bus source select
    always @(*) begin
        case(sb_src)
            REG_A: sb = a;
            REG_X: sb = x;
            REG_Y: sb = y;
            REG_S: sb = s;
            REG_ADD: sb = add;
            REG_DATA: sb = data;
            default: sb = 8'h0;
        endcase
        case(db_src)
            DB_DATA:    db = data;
            DB_PCL:     db = pc[7:0];
            DB_PCH:     db = pc[15:8];
            DB_S:       db = s;
            DB_P:       db = irq_event ? p : p | FL_BU; // set break flags unless irq 
            DB_A:       db = a;
            DB_SB:      db = sb;
            default:    db = 8'h0;
        endcase

        RW = !db_write || rst_event;
        dor = db;
    end

    //alu
    assign ai = ai_inv ? ~sb : sb;
    assign bi = bi_inv ? ~db : adl_bi ? adl : db;
    alu u_alu(
        .ai  (ai  ),
        .bi  (bi  ),
        .ci  (ci  ),
        .op  (alu_OP),
        .out (alu_out),
        .N   (aluN),
        .V   (aluV),
        .Z   (aluZ),
        .C   (aluC)
    );


    //registers
    always @(posedge i_clk ) begin
        if (i_rst) begin
            a <= 0;
            x <= 0;
            y <= 0;
            s <= 0;
            p <= INITIAL_STATUS;              //disable IRQ on reset
            pch <= 0;
            pcl <= 0;
            pc <= RST_VECTOR;
            rpop <= 0;
            rexec <= 0;
            add <= 8'b0;
            ipage_up <= 0;
            bpage_up <= 0;
            bpage_down <= 0;
            
        end else if(rdy) begin
            rpop <= pop;
            rexec <= exec;

            pcl <= adl;
            pch <= adh;
            pc <= rdy ? nextpc : pc;

            add <= holdalu ? add : alu_out;
            a <= sb_a ? sb : a;
            x <= sb_x ? sb : x;
            y <= sb_y ? sb : y;
            s <= sb_s ? sb : push ? s-1 : pop ? s+1 : s;
            p <= db_p ? (db & ~FL_BU) : ~clear_mask & (set_mask  | p);

            //update status flags for non-ALU register load/transfers
            if (load && (sb_a || sb_x || sb_y)) begin
                p[1] <= ~|sb;
                p[7] <= sb[7];
            end

            //update status flags for ALU ops
            if (exec) begin
                p[1] <= aluZ;               //set Z flag
                p[7] <= aluN;               //set N flag
                p[6] <= setV ? aluV : p[6]; //set V flag
                p[0] <= setC ? aluC : p[0]; //set C flag
            end

            // index math is unsigned, so we cross a page boundary if carry bit is set
            ipage_up <= aluC;

            // branch math: bi [unsigned base addr]  +  ai [signed offset], 
            bpage_up <= bi[7] & !ai[7] & !aluN; // crossed to next page if base>127, offset>0, and result <= 127
            bpage_down <= !bi[7] & ai[7] & aluN; // crossed to prev page if base<=127, offset<0, and result > 127

            if (handle_int && (irq_event | nmi_event)) p[2] <= 1; // set interrupt bit
            p[5] <= p[5];                       //never change bit 5
        end

        `ifdef DEBUG_REG
            if(reg_set_en) begin
                pc <= pc_set;
                s <= s_set;
                a <= a_set;
                x <= x_set;
                y <= y_set;
                p <= p_set;     
            end
        `endif
    end

    //state machine
    always @(posedge i_clk ) begin
        if (i_rst) begin
            state <= T1_DECODE;
        end
        else if (rdy) begin
            case(state)
            T0_FETCH:       state <= T1_DECODE;
            T1_DECODE:      state <= initial_state;
            T2_ZPG:         state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_ZPGXY:       state <= T3_ZPGXY;
            T3_ZPGXY:       state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_ABS:         state <= T3_ABS;
            T3_ABS:         state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_ABSXY:       state <= T3_ABSXY;
            T3_ABSXY:       state <= !skip ? T4_ABSXY : T0_FETCH;
            T4_ABSXY:       state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_XIND:        state <= T3_XIND;
            T3_XIND:        state <= T4_XIND;
            T4_XIND:        state <= T5_XIND;
            T5_XIND:        state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T2_INDY:        state <= T3_INDY;
            T3_INDY:        state <= T4_INDY;
            T4_INDY:        state <= !skip ? T5_INDY : rmw ? T_RMW_EXEC : T0_FETCH;
            T5_INDY:        state <= rmw ? T_RMW_EXEC : T0_FETCH;
            T_RMW_EXEC:     state <= T_RMW_STORE;
            T_RMW_STORE:    state <= T0_FETCH;
            T2_JUMP:        state <= T3_JUMP;
            T3_JUMP:        state <= T1_DECODE;
            T2_JUMPIND:     state <= T3_JUMPIND;
            T3_JUMPIND:     state <= T4_JUMPIND;
            T4_JUMPIND:     state <= T5_JUMPIND;
            T5_JUMPIND:     state <= T1_DECODE;
            T2_BRANCH:      state <= T3_BRANCH;
            T3_BRANCH:      state <= skip ? T1_DECODE : T4_BRANCH;
            T4_BRANCH:      state <= T1_DECODE;
            T2_PUSH:        state <= T0_FETCH;
            T2_POP:         state <= T3_POP;
            T3_POP:         state <= T0_FETCH;
            T2_BRK:         state <= T3_BRK;
            T3_BRK:         state <= T4_BRK;
            T4_BRK:         state <= T5_BRK;
            T5_BRK:         state <= T2_JUMP;
            T2_RTI:         state <= T3_RTI;
            T3_RTI:         state <= T4_RTI;
            T4_RTI:         state <= T5_RTI;
            T5_RTI:         state <= T3_JUMP;
            T2_RTS:         state <= T3_RTS;
            T3_RTS:         state <= T4_RTS;
            T4_RTS:         state <= T5_RTS;
            T5_RTS:         state <= T0_FETCH;
            T2_JSR:         state <= T3_JSR;
            T3_JSR:         state <= T4_JSR;
            T4_JSR:         state <= T5_JSR;
            T5_JSR:         state <= T3_JUMP;
            default:        state <= T_JAM;
            endcase
        end
    end

    // control
    always @(*) begin
        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 0;
        holdalu     = 0;
        push        = 0;
        pop         = 0;
        skip        = 0;
        exec        = 0;
        save        = 0;
        jump        = 0;
        jam         = 0;
        sync        = 0;
        handle_int  = 0;

        //default bus config
        db_src  = DB_DATA;
        sb_src  = idx_XY ? REG_X : REG_Y;

        //register write control 
        db_write=0;      // write db to memory
        sb_a=0;         // save sb -> reg A
        sb_x=0;         // save sb -> reg X
        sb_y=0;         // save sb -> reg Y
        sb_s=0;         // save sb -> reg s
        db_p=0;         // save db -> p
        adl_bi=0;

        // default ALU config
        alu_OP = ALU_ADD;
        ai_inv = 0;
        bi_inv = 0;
        ci = 0;

        case(state)
            T0_FETCH:   begin
                        sync = 1;               // signal new instruction (also increments pc)
                        exec = alu_en && !rmw;  // execute alu ops (except rmw)
                        save = load;            // save non-alu data to reg (includes loads, transfers, pulls) 
                        end

            T1_DECODE:  begin
                        inc_pc = !single_byte;  // increment pc unless this is single byte opcode
                        save = alu_en && !rmw;  // save alu result
                        end

            T2_ZPG:     begin
                        adl_src = ADDR_DATA;    // data address at {00, low }
                        adh_src = ADDR_Z;
                        save = store;           // write data on store ops
                        end

            T2_ZPGXY:   begin
                        adl_src = ADDR_HOLD;    // hold addr while computing low addr from base + index
                        adh_src = ADDR_HOLD;
                        end

            T3_ZPGXY:   begin
                        adl_src = ADDR_ADD;     // data address at {00, base + index}
                        adh_src = ADDR_Z;
                        save = store;           // write data on store ops
                        end

            T2_JUMP,
            T2_JUMPIND,
            T2_ABS:     begin
                        sb_src = REG_Z;         // compute (LL+0) to hold low address in alu 
                        inc_pc = 1;             // fetch high address
                        end+

            T3_ABS:     begin
                        adl_src = ADDR_ADD;     // data address at {high,low}
                        adh_src = ADDR_DATA;
                        save = store;           // write data on store ops
                        end

            T_BOOT,
            T2_ABSXY:   begin
                                                // compute low addr from base + index
                        inc_pc = 1;             // fetch high addr
                        end                

            T3_ABSXY,
            T4_INDY:    begin
                        adl_src = ADDR_ADD;     // data address at {high,low} (assumes no carry)
                        adh_src = ADDR_DATA;
                        if (ipage_up || store || rmw) begin
                            sb_src = REG_Z;     // save base address in alu, adding zero (with carry if overflow)
                            ci = ipage_up;      // low = base + index overflowed, need to increment page
                        end else begin
                            skip = 1;           // no carry, we can skip next state
                        end
                        end

            T4_ABSXY,
            T5_INDY:    begin
                        adl_src = ADDR_HOLD;    // data address at {high,low} (now with incremented page)
                        adh_src = ADDR_ADD;
                        save = store;           // write data on store ops
                        end

            T2_XIND:    begin
                        adl_src = ADDR_DATA;    // fetch base address while computing base + index (data discarded) 
                        adh_src = ADDR_Z;
                        end                
            T3_XIND:    begin
                        adl_src = ADDR_ADD;     // fetch low address = base+index 
                        adh_src = ADDR_Z;
                        sb_src = REG_ADD;       // increment base+index+1 for high address location
                        ci = 1;
                        db_src = DB_Z;
                        end
            T4_XIND:    begin
                        sb_src = REG_Z;         // read low address and hold in alu by computing (low+0)
                        adl_src = ADDR_ADD;     // fetch high address at base+index+1
                        adh_src = ADDR_Z;
                        end
            T5_XIND:    begin
                        adl_src = ADDR_ADD;     // data address at {high,low}
                        adh_src = ADDR_DATA;
                        save = store;           // write data on store ops
                        end

            T2_INDY:    begin
                        adl_src = ADDR_DATA;    // fetch low base address 
                        adh_src = ADDR_Z;
                        sb_src = REG_Z;         // increment pointer to location of high base address
                        ci = 1;
                        end

            T3_INDY:    begin
                                                // read low base address and compute low addr = base + index
                        adl_src = ADDR_ADD;     // fetch high base address 
                        adh_src = ADDR_Z;
                        end

            // T4_INDY-T5_INDY handled with T3_ABSXY-T4_ABSXY

            T_RMW_EXEC: begin
                        exec = 1;               // execute rmw operation
                        adl_src = ADDR_HOLD;    // hold onto address
                        adh_src = ADDR_HOLD;
                        end

            T_RMW_STORE:begin
                        adl_src = ADDR_HOLD;    
                        adh_src = ADDR_HOLD;
                        save = 1;               // store rmw result in same location
                        end

            T2_BRANCH:  begin
                        adl_bi = 1;             // load the current low address (pc) into the alu
                        sb_src = REG_DATA;      // compute branch location = pc+data
                        end

            T3_BRANCH:  begin
                        adl_src = ADDR_ADD;     // new pc assuming we haven't crossed a page
                        adh_src = ADDR_HOLD;    
                        jump = 1;               // load pc from address (i.e. jump)
                        if (bpage_up) begin
                            db_src = DB_PCH;    // we jumped to next page, increment pch
                            sb_src = REG_Z;
                            ci = 1;
                        end else if (bpage_down) begin
                            db_src = DB_PCH;    // we jumped to prev page, decrement pch
                            sb_src = REG_Z;
                            ai_inv = 1;
                        end else begin
                            inc_pc = 1;         // still on same page, this cycle becomes T0_FETCH
                            sync = 1;
                            skip = 1;
                        end
                        end

            T4_BRANCH:  begin
                        adl_src = ADDR_HOLD;    // now we are on the currect page, jump to new pc
                        adh_src = ADDR_ADD;
                        jump = 1;
                        sync = 1;               // this cycle becomes T0_FETCH
                        end

            T3_JUMP:    begin
                        adl_src = ADDR_ADD;     // load new pc
                        adh_src = ADDR_DATA;    
                        jump = 1;               // update pc from address
                        sync = 1;               // this is fetch stage.
                        end

            T3_JUMPIND: begin
                        adl_src = ADDR_ADD;     // fetch low pc
                        adh_src = ADDR_DATA;
                        adl_bi = 1;             // load the current low address into the alu
                        sb_src = REG_Z;         // 
                        ci = 1;
                        end
            T4_JUMPIND: begin
                        adl_src = ADDR_ADD;     // fetch high pc
                        adh_src = ADDR_HOLD;    
                        sb_src = REG_Z;         // compute (LL+0) to hold low pc in alu 
                        end
            T5_JUMPIND: begin
                        adl_src = ADDR_ADD;     // jump to new pc
                        adh_src = ADDR_DATA;    
                        jump = 1;               // update pc from address
                        sync = 1;               // this is fetch stage.
                        end

            T2_PUSH:    begin
                        save = 1;               // save data to stack
                        push = 1;               // update stack pointer
                        end

            T2_POP:     pop = 1;                // update stack pointer
            T3_POP:     begin end               // load data from stack (handled automatically from pop signal)

            T2_BRK:     begin
                        db_src = DB_PCH;        // push pch to stack
                        db_write = 1;
                        push = 1;
                        end
            T3_BRK:     begin
                        db_src = DB_PCL;        // push pcl to stack
                        db_write = 1;
                        push = 1;
                        end
            T4_BRK:     begin
                        db_src = DB_P;          // push status register to stack
                        db_write = 1;
                        push = 1;
                        end
            T5_BRK:     begin
                        adl_src = ADDR_INT;     // jump to interrupt register
                        adh_src = ADDR_INT;
                        jump = 1;
                        handle_int = 1;
                        end


            // for rts/rti, initial states are popping things off the stack
            // pops take 3 cycles to complete, so initial states are common...
            T2_RTS,                             // T2_RTS: pop pcl
            T3_RTS,                             // T3_RTS: fetch pcl, pop pch
            T2_RTI,                             // T2_RTI: pop p
            T3_RTI:                             // T3_RTI: fetch p, pop pcl
                        pop = 1;

            T4_RTI:     begin
                        db_p = 1;               //store p, fetch pcl, pop pch
                        pop = 1;
                        end
            T4_RTS,
            T5_RTI:     sb_src = REG_Z;         // store pcl in alu, fetch pch

            T5_RTS:    begin
                        adl_src = ADDR_ADD;     // fetch new pc
                        adh_src = ADDR_DATA;    
                        jump = 1;               // update pc from address
                        end

            T2_JSR:     sb_src = REG_Z;         // read new pcl and store in alu (compute pcl+0)
            T3_JSR:     begin
                        holdalu = 1;            // continue to hold pcl in alu
                        db_src = DB_PCH;        // push old pch
                        db_write=1;                        
                        push = 1;
                        end

            T4_JSR:    begin
                        holdalu = 1;            // continue to hold new pcl in alu
                        db_src = DB_PCL;        // push old pcl
                        db_write=1;                        
                        push = 1;
                        end
            T5_JSR:     begin
                                                // load new pch
                        holdalu = 1;            // continue to hold new pcl in alu
                        end
            // state then transfers to T3_JUMP, where {pch, pcl} -> pc

            default:    jam = 1;
        endcase

        if (push || rpop) begin
            adl_src = ADDR_STACK;
            adh_src = ADDR_STACK;
        end

        if (exec) begin
            //configure the alu and bus for this instruction
            alu_OP = op_alu_OP;
            ai_inv = op_ai_inv;
            bi_inv = op_bi_inv;
            ci = (op_Pci && p[0]) || op_ci;
            sb_src = op_sb_src;
            db_src = op_db_src;
        end

        //save result to register or memory
        if (save) begin
            case (op_dst)
                REG_DATA:   db_write = 1;
                REG_A:      sb_a=1;
                REG_X:      sb_x=1;
                REG_Y:      sb_y=1;
                REG_S:      sb_s=1;
                REG_P:      db_p=1;
                default:    begin end
            endcase
            sb_src = alu_en ? REG_ADD : op_sb_src;
            db_src = (db_write && (op_db_src != DB_P)) ? DB_SB : op_db_src;
        end

    end

endmodule

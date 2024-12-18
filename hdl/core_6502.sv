`timescale 1ns/1ps
`include "6502_defs.vh"

// 6502 uses level-triggered logic with latches that are updated during clk=1
// FPGA uses edge-triggered logic with registers loaded on posedge(clk)
// thus latches loaded during clk_m1 phase should be implemented as registers triggered on posedge(m2) and vice versa 

// unlike 6502 latches, registers are not transparent, cannot forward updated state on same clock,
// so several in several cases we will need special logic

// 2-byte, 2 cycle opcode summary:
// https://www.nesdev.org/wiki/Visual6502wiki/6502_Timing_States
//
//      OP 1                            OP 2
//      ----------------------------    ---------------
// m1   T0: PC => addr                   
// m2   T0: bus => data_reg              
// m1   T1: data => ir, PC+1 => addr
// m2   T1: bus => data_reg, decode
// m1   T2: data_reg => alu (via DB)    T0: PC => addr
// m2   T2: alu=>add                    T0: bus => data_reg
// m1     : add=>a                      T1: data => ir, PC+1 => addr
// m2                                   T1: bus => data_reg, decode
// m1                                   T2: data_reg => alu (via DB)
// m2                                   T2: alu=>add
// m1                                       add=>a

module core_6502 #(
    parameter NMI_VECTOR=16'hfffa,
    parameter RST_VECTOR=16'hfffc,
    parameter IRQ_VECTOR=16'hfffe,
    parameter INITIAL_STATUS=FL_I | FL_U,
    parameter INITIAL_STACK=8'hfd
    )
    (
    input  logic clk_m1,
    input  logic clk_m2,
    input  logic rst,
    input  logic [7:0] data_i,
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
    input  logic [15:0] pc_set,
    input  logic [7:0] s_set,
    input  logic [7:0] a_set,
    input  logic [7:0] x_set,
    input  logic [7:0] y_set,
    input  logic [7:0] p_set,
    output  logic [15:0] pc_dbg,
    output  logic [7:0] s_dbg,
    output  logic [7:0] a_dbg,
    output  logic [7:0] x_dbg,
    output  logic [7:0] y_dbg,
    output  logic [7:0] p_dbg
    `endif 

    );


    // ready signal
    wire rdy = READY | ~RW; //ignore not ready when writing


    // registers
    logic [7:0] ir, add;
    logic [7:0] a, s, x, y, p /*verilator public_flat*/;

      // state
    logic [5:0] state, initial_state;

    //alu
    logic [3:0] alu_OP;
    logic [7:0] ai, bi;
    logic [7:0] alu_out;
    logic ci, aluN, aluV, aluZ, aluC;
    logic ai_inv, bi_inv, adl_bi;

    // decode 
    logic single_byte, two_cycle;
    logic idx_XY;           // select index X(1) or Y(0) for abs,X/Y and zpg,X/Y ops
    logic mem_read, mem_write, reg_write;
    logic alu_en;
    logic [3:0] op_alu_OP;
    logic op_ai_inv;   // alu ai src
    logic op_bi_inv;   // alu bi src
    logic op_Pci, op_ci;       // carry in = ci || (Pci & p[0])
    logic [2:0] op_db_src,op_sb_src,op_dst;
    logic setV, setC;    // update p C or V bits with alu result

   // control signals
    logic [2:0] db_src,sb_src,dst;
    logic [2:0] adl_src,adh_src;
    logic update_addrl,update_addrh;
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

    // incoming data register
    logic [7:0] data;
    always_ff @(posedge clk_m1) begin
        if (rst) begin
            data <= 0;
        end else begin
            data <= data_i;
            // TODO: when to we need this?
            // if(rdy) data <= data_i;
        end
    end

    // busses
    // db: data bus: typically memory read, but can be addr_lo or zero
    // sb: secondry bus: read from registers (a,x,y,s, zero)
    logic [7:0] db, sb;

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

        RW = !(save && op_dst == REG_DATA) || rst_event;
        dor = RW ? 0 : result;
    end


    // ADDRESS BUS
    // external address bus updated on m1
    always @(posedge clk_m1 ) begin
        if (rst) begin
            addr <= 0;
        end else begin
            if (update_addrh) addr[15:8] <= adh;
            if (update_addrl) addr[7:0] <= adl;

            if(reg_set_en) addr <= pc_set;

        end
    end
    // assign addr = {adh, adl};
    logic [7:0] adl,adh;
    always @(*) begin
        case(adl_src)
            ADDR_DATA: adl = data;
            ADDR_ADD: adl = add;
            ADDR_Z: adl = 8'b0;
            ADDR_INT: adl = rst_event ? RST_VECTOR[7:0] :
                            nmi_event ? NMI_VECTOR[7:0] :
                            IRQ_VECTOR[7:0];
            ADDR_STACK: adl = s;
            default: adl = pcl; //ADDR_PC
        endcase
        case(adh_src)
            ADDR_DATA: adh = data;
            ADDR_ADD: adh = add;
            ADDR_Z: adh = 8'b0;
            ADDR_INT: adh = rst_event ? RST_VECTOR[15:8] :
                            nmi_event ? NMI_VECTOR[15:8] :
                            IRQ_VECTOR[15:8];
            ADDR_STACK: adh = STACKPAGE;
            default: adh = pch; //ADDR_PC
        endcase
    end

    // always @(posedge clk_m2 ) begin
    //     if (rst) begin
    //         adl <= 0;
    //         adh <= 0;
    //     end else begin
    //         case(adl_src)
    //             ADDR_DATA: adl <= data;
    //             ADDR_ADD: adl <= add;
    //             ADDR_Z: adl <= 8'b0;
    //             ADDR_INT: adl <= rst_event ? RST_VECTOR[7:0] :
    //                             nmi_event ? NMI_VECTOR[7:0] :
    //                             IRQ_VECTOR[7:0];
    //             ADDR_STACK: adl <= s;
    //             default: adl <= pcl; //ADDR_PC
    //         endcase
    //         case(adh_src)
    //             ADDR_DATA: adh <= data;
    //             ADDR_ADD: adh <= add;
    //             ADDR_Z: adh <= 8'b0;
    //             ADDR_INT: adh <= rst_event ? RST_VECTOR[15:8] :
    //                             nmi_event ? NMI_VECTOR[15:8] :
    //                             IRQ_VECTOR[15:8];
    //             ADDR_STACK: adh <= STACKPAGE;
    //             default: adh <= pch; //ADDR_PC
    //         endcase
    //     end
    // end

    // PC (updated on m1)
    logic [7:0] pcl, pch;   // low and high byte of *incremented* pc
    logic [15:0] pc, pcinc /*verilator public_flat*/;
    assign pcinc = inc_pc ? pc+1 : pc;
    assign pcl = pcinc[7:0];
    assign pch = pcinc[15:8];
    always @(posedge clk_m1 ) begin
        if (rst) begin
            pc <= RST_VECTOR;
        end else begin

            if ((nmi_event || irq_event) && sync) begin
                pc <= pc;
            end else if (jump) begin
                pc <= {adh, adl};
            end else begin
                pc <= pcinc;
            end

        `ifdef DEBUG_REG
            if(reg_set_en) begin
                pc <= pc_set;
            end
        `endif
        end
    end

    //instruction pointer: pc of current opcode
    //unused by core, but helpful for debug
    (* mark_debug = "true" *)  logic [15:0] ip;
    always @(posedge clk_m1 ) begin
        if (rst) ip <= RST_VECTOR;
        else if (sync && rdy) ip <= addr;
    end


    logic nmi_event, nmi_handled, irq_event, rst_event;
    logic fetch_intr;
    wire IRQ_masked = IRQ && !p[2];
    always @(posedge clk_m1 ) begin
        if (rst) begin
            nmi_event <= 0;
            irq_event <= 0;
            rst_event <= 1;
            nmi_handled <= 0;
            fetch_intr <= 0;
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
    always @(posedge clk_m1 ) begin
        if (rst) ir <= 0;  //break from RESET_VECTOR
        // else if (sync && rdy)  ir <= fetch_intr ? 0 : data;
        else if (sync && rdy)  ir <= data_i;
    end


    // decode instruction
    decode u_decode(
        .clk        (clk_m1         ),
        .rst           (rst         ),
        .rdy           (rdy),
        .opcode        (ir        ),
        .pstatus       (p             ),
        .initial_state (initial_state ),
        .single_byte   (   ),
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

    // predecode signals
    assign two_cycle =  (data_i ==? 8'b1??_000_?0) ||     // LD/CP imm
                        (data_i ==? 8'b???_010_??) ||     // imm or impl
                        (data_i ==? 8'b???_110_?0);       // impl
    assign single_byte = (data_i == 8'h0) || (data_i ==? 8'b???_?10_?0);

    //registers
    logic [7:0] result;
    always @(*) begin
        case(op_sb_src)
            REG_A: result = a;
            REG_X: result = x;
            REG_Y: result = y;
            REG_S: result = s;
            REG_ADD: result = add;
            REG_DATA: result = data;
            default: result = 8'h0;
        endcase
        if (alu_en) result = add;
    end

    always @(posedge clk_m1 ) begin
        if (rst) begin
            a <= 0;
            x <= 0;
            y <= 0;
            s <= INITIAL_STACK;
            p <= INITIAL_STATUS;              //disable IRQ on reset
            rpop <= 0;
            rexec <= 0;
            // add <= 8'b0;
            ipage_up <= 0;
            bpage_up <= 0;
            bpage_down <= 0;
            
        end else if(rdy) begin
            rpop <= pop;
            rexec <= exec;

            // set / clear flags
            p <= ~clear_mask & (set_mask  | p);

            // TODO: do this differently...
            if(push) s <= s-1;
            if(pop) s <= s+1;

            //save result to registers
            if (save) begin
                case (op_dst)
                    REG_A:      a <= result;
                    REG_X:      x <= result;
                    REG_Y:      y <= result;
                    REG_S:      s <= result;
                    REG_P:      p <= (db & ~FL_BU);
                    default:    begin end
                endcase    
                // db_src = (db_write && (op_db_src != DB_P)) ? DB_SB : op_db_src;
            end


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
                s <= s_set;
                a <= a_set;
                x <= x_set;
                y <= y_set;
                p <= p_set;     
            end
        `endif
    end

    always @(posedge clk_m2 ) begin
        if (rst) add <= 0;
        else if (!holdalu) add <= alu_out;
    end

    //alu
    logic [1:0] ai_src, bi_src;
    always_comb begin
        case(ai_src)
            ADD_BUS:    ai = sb;
            ADD_INVBUS: ai = ~sb;
            ADD_Z:  ai = 0;
            ADD_N:  ai = 8'hff;
        endcase
        case(bi_src)
            ADD_BUS:    bi = adl_bi ? adl : db;
            ADD_INVBUS: bi = ~db;
            ADD_Z:  bi = 0;
            ADD_N:  bi = 8'hff;
        endcase
    end

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

    //state machine
    logic Tlast;
    always @(posedge clk_m1 ) begin
        if (rst) begin
            // state <= T2_DECODE;
            state <= T1;
        end
        else if (rdy) begin
            case(state)

                T0:       state <= T1;
                T1:       state <= T2;
                T2:       state <= T3;
                T3:       state <= T4;
                T4:       state <= T5;
                T5:       state <= T6;
                T6:       state <= T_JAM;

            // T0_EXEC:       state <= T1_FETCH;
            // T1_FETCH:       state <= two_cycle ? T0_EXEC : initial_state;
            // // T2_DECODE:      state <= two_cycle ? T1_FETCH : initial_state;
            // T2_ZPG:         state <= rmw ? T_RMW_EXEC : T0_EXEC;
            // T2_ZPGXY:       state <= T3_ZPGXY;
            // T3_ZPGXY:       state <= rmw ? T_RMW_EXEC : T0_EXEC;
            // T2_ABS:         state <= T3_ABS;
            // T3_ABS:         state <= rmw ? T_RMW_EXEC : T0_EXEC;
            // T2_ABSXY:       state <= T3_ABSXY;
            // T3_ABSXY:       state <= !skip ? T4_ABSXY : T0_EXEC;
            // T4_ABSXY:       state <= rmw ? T_RMW_EXEC : T0_EXEC;
            // T2_XIND:        state <= T3_XIND;
            // T3_XIND:        state <= T4_XIND;
            // T4_XIND:        state <= T5_XIND;
            // T5_XIND:        state <= rmw ? T_RMW_EXEC : T0_EXEC;
            // T2_INDY:        state <= T3_INDY;
            // T3_INDY:        state <= T4_INDY;
            // T4_INDY:        state <= !skip ? T5_INDY : rmw ? T_RMW_EXEC : T0_EXEC;
            // T5_INDY:        state <= rmw ? T_RMW_EXEC : T0_EXEC;
            // T_RMW_EXEC:     state <= T_RMW_STORE;
            // T_RMW_STORE:    state <= T0_EXEC;
            // T2_JUMP:        state <= T3_JUMP;
            // T3_JUMP:        state <= T1_FETCH;
            // T2_JUMPIND:     state <= T3_JUMPIND;
            // T3_JUMPIND:     state <= T4_JUMPIND;
            // T4_JUMPIND:     state <= T5_JUMPIND;
            // T5_JUMPIND:     state <= T1_FETCH;
            // T2_BRANCH:      state <= T3_BRANCH;
            // T3_BRANCH:      state <= skip ? T1_FETCH : T4_BRANCH;
            // T4_BRANCH:      state <= T1_FETCH;
            // T2_PUSH:        state <= T0_EXEC;
            // T2_POP:         state <= T3_POP;
            // T3_POP:         state <= T0_EXEC;
            // T2_BRK:         state <= T3_BRK;
            // T3_BRK:         state <= T4_BRK;
            // T4_BRK:         state <= T5_BRK;
            // T5_BRK:         state <= T2_JUMP;
            // T2_RTI:         state <= T3_RTI;
            // T3_RTI:         state <= T4_RTI;
            // T4_RTI:         state <= T5_RTI;
            // T5_RTI:         state <= T3_JUMP;
            // T2_RTS:         state <= T3_RTS;
            // T3_RTS:         state <= T4_RTS;
            // T4_RTS:         state <= T5_RTS;
            // T5_RTS:         state <= T0_EXEC;
            // T2_JSR:         state <= T3_JSR;
            // T3_JSR:         state <= T4_JSR;
            // T4_JSR:         state <= T5_JSR;
            // T5_JSR:         state <= T3_JUMP;

            default:        state <= T_JAM;
            endcase

            if (Tlast) state <= T0;
            if (jam) state <= T_JAM;


            `ifdef DEBUG_REG
                if(reg_set_en) begin
                    state <= T1_DEBUG;
                end
            `endif

        end

    end

    //TODO:
    wire op_jsr = initial_state == T3_JSR;

    logic inc_addr, dec_addr;
    logic inc_bi, dec_bi, inc_ai, dec_ai;
    // control
    always @(*) begin
        adl_src     = ADDR_PC;
        adh_src     = ADDR_PC;
        inc_pc      = 1;
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
        update_addrl = 1;
        update_addrh = 1;

        inc_addr = 0;
        dec_addr = 0;
        inc_bi = 0;
        dec_bi = 0;
        inc_ai = 0;
        dec_ai = 0;

        //default bus config
        db_src  = DB_DATA;
        sb_src  = idx_XY ? REG_X : REG_Y;

        adl_bi=0;

        // default ALU config
        alu_OP = ALU_ADD;
        ai_inv = 0;
        bi_inv = 0;
        ci = 0;
        ai_src = ADD_BUS;
        bi_src = ADD_BUS;

        Tlast=0;
        jam = 0;
        // From: https://www.nesdev.org/wiki/Visual6502wiki/6502_Timing_States
        // The convention for presenting time states for instruction execution here
        // is a a little different from that supplied in the usual programming literature in two ways:
        // by time numbering and by first through last cycles listed.
        // The time numbering issue applies to matching up the time codes used in Appendix A
        // of "MCS6500 Microcomputer Family Hardware Manual", with the time states documented here.
        // "T0" in the manual matches with the states that have [T1] in them here (most often T+ [T1]).
        // The rest of the time codes in the hardware manual listings match up with those here after being incremented by one.

        // The second-to-last time code in each hardware manual listing corresponds to the T0-containing states here,
        // and the last code in each listing is the [T1] state again.         
        
        case (state)
            T0: begin
                // fetch data
                end
            T1: begin
                // fetch opcode
                sync = 1;
                exec = 1; // configure alu
                save = 1; // register result
                Tlast = two_cycle;
                end
            T1_DEBUG: begin
                sync = 1;
                Tlast = two_cycle;
                end
            T_JAM: begin
                inc_pc = 0;
                // $display("6502 jammed at pc=0x%4h", pc);
            end
            default: begin
                jam = 1;
                inc_pc = 0;
            end                
        endcase

        // case(state)

        //     T0_EXEC:    begin
        //                 exec = 1;
        //                 inc_pc = 1;
        //                 end

        //     T1_FETCH:   begin
        //                 sync = 1;               // signal new instruction (also increments pc)
        //                 inc_pc = 1;
        //                 save = !rmw;  // save alu result
        //                 end

        //     // T2_DECODE:  begin

        //     //             inc_pc = !single_byte;  // increment pc unless this is single byte opcode
        //     //             exec = two_cycle;

        //     //                 if(op_jsr) begin
        //     //                     adl_src = ADDR_STACK;
        //     //                     adh_src = ADDR_STACK;
        //     //                 end
        //     //             end

        //     T2_ZPG:     begin
        //                 adl_src = ADDR_DATA;    // data address at {00, low }
        //                 adh_src = ADDR_Z;
        //                 save = store;           // write data on store ops
        //                 end

        //     T2_ZPGXY:   begin
        //                 update_addrl = 0;        // hold addr while computing low addr from base + index
        //                 update_addrh = 0;
        //                 end

        //     T3_ZPGXY:   begin
        //                 adl_src = ADDR_ADD;     // data address at {00, base + index}
        //                 adh_src = ADDR_Z;
        //                 save = store;           // write data on store ops
        //                 end

        //     T2_JUMP,
        //     T2_JUMPIND,
        //     T2_ABS:     begin
        //                 sb_src = REG_Z;         // compute (LL+0) to hold low address in alu 
        //                 inc_pc = 1;             // fetch high address
        //                 end

        //     T3_ABS:     begin
        //                 adl_src = ADDR_ADD;     // data address at {high,low}
        //                 adh_src = ADDR_DATA;
        //                 save = store;           // write data on store ops
        //                 end

        //     T_BOOT,
        //     T2_ABSXY:   begin
        //                                         // compute low addr from base + index
        //                 inc_pc = 1;             // fetch high addr
        //                 end                

        //     T3_ABSXY,
        //     T4_INDY:    begin
        //                 adl_src = ADDR_ADD;     // data address at {high,low} (assumes no carry)
        //                 adh_src = ADDR_DATA;
        //                 if (ipage_up || store || rmw) begin
        //                     sb_src = REG_Z;     // save base address in alu, adding zero (with carry if overflow)
        //                     ci = ipage_up;      // low = base + index overflowed, need to increment page
        //                 end else begin
        //                     skip = 1;           // no carry, we can skip next state
        //                 end
        //                 end

        //     T4_ABSXY,
        //     T5_INDY:    begin
        //                 update_addrl = 0;        // data address at {high,low} (now with incremented page)
        //                 update_addrh = 0;
        //                 save = store;           // write data on store ops
        //                 end

        //     T2_XIND:    begin
        //                 adl_src = ADDR_DATA;    // fetch base address while computing base + index (data discarded) 
        //                 adh_src = ADDR_Z;
        //                 end                
        //     T3_XIND:    begin
        //                 adl_src = ADDR_ADD;     // fetch low address = base+index 
        //                 adh_src = ADDR_Z;
        //                 sb_src = REG_ADD;       // increment base+index+1 for high address location
        //                 ci = 1;
        //                 db_src = DB_Z;
        //                 end
        //     T4_XIND:    begin
        //                 sb_src = REG_Z;         // read low address and hold in alu by computing (low+0)
        //                 adl_src = ADDR_ADD;     // fetch high address at base+index+1
        //                 adh_src = ADDR_Z;
        //                 end
        //     T5_XIND:    begin
        //                 adl_src = ADDR_ADD;     // data address at {high,low}
        //                 adh_src = ADDR_DATA;
        //                 save = store;           // write data on store ops
        //                 end

        //     T2_INDY:    begin
        //                 adl_src = ADDR_DATA;    // fetch low base address 
        //                 adh_src = ADDR_Z;
        //                 sb_src = REG_Z;         // increment pointer to location of high base address
        //                 ci = 1;
        //                 end

        //     T3_INDY:    begin
        //                                         // read low base address and compute low addr = base + index
        //                 adl_src = ADDR_ADD;     // fetch high base address 
        //                 adh_src = ADDR_Z;
        //                 end

        //     // T5_INDY-T6_INDY handled with T4_ABSXY-T5_ABSXY

        //     T_RMW_EXEC: begin
        //                 exec = 1;               // execute rmw operation
        //                 update_addrl = 0;        // hold onto address
        //                 update_addrh = 0;
        //                 end

        //     T_RMW_STORE:begin
        //                 update_addrl = 0;        // hold onto address 
        //                 update_addrh = 0;
        //                 save = 1;               // store rmw result in same location
        //                 end

        //     T3_BRANCH:  begin
        //                 adl_bi = 1;             // load the current low address (pc) into the alu
        //                 sb_src = REG_DATA;      // compute branch location = pc+data
        //                 end

        //     // T3_BRANCH:  begin
        //     //             adl_src = ADDR_ADD;     // new pc assuming we haven't crossed a page
        //     //             update_addrh = 0;    
        //     //             jump = 1;               // load pc from address (i.e. jump)
        //     //             if (bpage_up) begin
        //     //                 db_src = DB_PCH;    // we jumped to next page, increment pch
        //     //                 sb_src = REG_Z;
        //     //                 ci = 1;
        //     //             end else if (bpage_down) begin
        //     //                 db_src = DB_PCH;    // we jumped to prev page, decrement pch
        //     //                 sb_src = REG_Z;
        //     //                 ai_inv = 1;
        //     //             end else begin
        //     //                 inc_pc = 1;         // still on same page, this cycle becomes T1_FETCH
        //     //                 sync = 1;
        //     //                 skip = 1;
        //     //             end
        //     //             end

        //     T4_BRANCH:  begin
        //                 update_addrl = 0;       // now we are on the currect page, jump to new pc
        //                 adh_src = ADDR_ADD;
        //                 jump = 1;
        //                 sync = 1;               // this cycle becomes T1_FETCH
        //                 end

        //     T3_JUMP:    begin
        //                 adl_src = ADDR_ADD;     // load new pc
        //                 adh_src = ADDR_DATA;    
        //                 jump = 1;               // update pc from address
        //                 sync = 1;               // this is fetch stage.
        //                 end

        //     T3_JUMPIND: begin
        //                 adl_src = ADDR_ADD;     // fetch low pc
        //                 adh_src = ADDR_DATA;
        //                 adl_bi = 1;             // load the current low address into the alu
        //                 sb_src = REG_Z;         // 
        //                 ci = 1;
        //                 end
        //     T4_JUMPIND: begin
        //                 adl_src = ADDR_ADD;     // fetch high pc
        //                 update_addrh = 0;    
        //                 sb_src = REG_Z;         // compute (LL+0) to hold low pc in alu 
        //                 end
        //     T5_JUMPIND: begin
        //                 adl_src = ADDR_ADD;     // jump to new pc
        //                 adh_src = ADDR_DATA;    
        //                 jump = 1;               // update pc from address
        //                 sync = 1;               // this is fetch stage.
        //                 end

        //     T2_PUSH:    begin
        //                 save = 1;               // save data to stack
        //                 push = 1;               // update stack pointer
        //                 end

        //     T2_POP:     pop = 1;                // update stack pointer
        //     T3_POP:     begin end               // load data from stack (handled automatically from pop signal)

        //     T2_BRK:     begin
        //                 db_src = DB_PCH;        // push pch to stack
        //                 // db_write = 1;
        //                 push = 1;
        //                 end
        //     T3_BRK:     begin
        //                 db_src = DB_PCL;        // push pcl to stack
        //                 // db_write = 1;
        //                 push = 1;
        //                 end
        //     T4_BRK:     begin
        //                 db_src = DB_P;          // push status register to stack
        //                 // db_write = 1;
        //                 push = 1;
        //                 end
        //     T5_BRK:     begin
        //                 adl_src = ADDR_INT;     // jump to interrupt register
        //                 adh_src = ADDR_INT;
        //                 jump = 1;
        //                 handle_int = 1;
        //                 end


        //     // for rts/rti, initial states are popping things off the stack
        //     // pops take 3 cycles to complete, so initial states are common...
        //     T2_RTS,                             // T3_RTS: pop pcl
        //     T3_RTS,                             // T4_RTS: fetch pcl, pop pch
        //     T2_RTI,                             // T3_RTI: pop p
        //     T3_RTI:                             // T4_RTI: fetch p, pop pcl
        //                 pop = 1;

        //     T4_RTI:     begin
        //                 db_p = 1;               //store p, fetch pcl, pop pch
        //                 pop = 1;
        //                 end
        //     T4_RTS,
        //     T5_RTI:     sb_src = REG_Z;         // store pcl in alu, fetch pch

        //     T5_RTS:    begin
        //                 adl_src = ADDR_ADD;     // fetch new pc
        //                 adh_src = ADDR_DATA;    
        //                 jump = 1;               // update pc from address
        //                 end


        //     // JSR
        //     // T1:          [pc] fetch op
        //     // T2:          [pc+1] fetch addr_low
        //     // T3:          [s], stash addr_low in s
        //     // T4:          [s], push pch
        //     // T5:          [s-1], push pcl
        //     // T6:          [pc+2], fetch addr_high
        //     // T0:          [addr], fetch next opcode

        //     // T2_JSR:     begin
        //     //             // fetch low addr (pc+1)
        //     //             // point at stack (on bus in T3)
        //     //             // adl <- s
        //     //             // adh <- 0
        //     //             end

        //     T2_JSR:     begin
        //                 inc_pc=0; // hold pc

        //                 // stash addr_low in s (db -> sb -> s)
        //                 sb_src=REG_DATA;
        //                 sb_s=1;

        //                 // addr stack
        //                 dec_addr=1;
        //                 end

        //     T3_JSR:     begin
        //                 dec_addr=1;

        //                 // holdalu = 1;            // continue to hold pcl in alu
        //                 // db_src = DB_PCH;        // push old pch
        //                 // db_write=1;                        
        //                 // push = 1;
        //                 end


        //     // push:       begin
        //     //             adl_src = ALU;
        //     //             adh_src = 0;
        //     //             alu_src = adl;
        //     //             alu_op = DEC;
        //     //             end

        //     // T4_JSR:     begin
        //     //             //alu -> adl
        //     //             //write pch (push)
        //     //             //set up decrement adl on alu (push)
        //     //             end

        //     // T5_JSR:    begin
        //     //             //alu -> adl
        //     //             //write pcl (push)
        //     //     holdalu = 1;            // continue to hold new pcl in alu
        //     //             db_src = DB_PCL;        // push old pcl
        //     //             db_write=1;                        
        //     //             push = 1;
        //     //             end
        //     // T6_JSR:     begin
        //     //                                     // load new pch
        //     //             holdalu = 1;            // continue to hold new pcl in alu
        //     //             end


        //     // T3_JSR:     begin
        //     //             sb_src = REG_Z;         // read new pcl and store in alu (compute pcl+0)
        //     //             end

        //     T4_JSR:    begin
        //                 holdalu = 1;            // continue to hold new pcl in alu
        //                 db_src = DB_PCL;        // push old pcl
        //                 // db_write=1;                        
        //                 push = 1;
        //                 end
        //     T5_JSR:     begin
        //                                         // load new pch
        //                 holdalu = 1;            // continue to hold new pcl in alu
        //                 end
        //     // state then transfers to T4_JUMP, where {pch, pcl} -> pc

        //     `ifdef DEBUG_REG
        //         T1_DEBUG: begin
        //                     sync = 1;
        //                 end
        //     `endif

        //     default:    jam = 1;
        // endcase

        // if (push || rpop) begin
        //     adl_src = ADDR_STACK;
        //     adh_src = ADDR_STACK;
        // end

        if (exec) begin
            //configure the alu and bus for this instruction
            alu_OP = op_alu_OP;
            ai_inv = op_ai_inv;
            bi_inv = op_bi_inv;
            ci = (op_Pci && p[0]) || op_ci;
            sb_src = op_sb_src;
            db_src = op_db_src;
        end


        if (inc_addr) begin
            adl_bi = 1;
            inc_bi = 1;
            adl_src = ADDR_ADD;
        end
        if (dec_addr) begin
            adl_bi = 1;
            dec_bi = 1;
            adl_src = ADDR_ADD;
        end

        // program alu for inc/dec ops
        if (inc_bi) begin
            ai_src = ADD_Z;
            bi_src = ADD_BUS;
            ci = 1;
            alu_OP = ALU_ADD;
        end else if (dec_bi) begin
            ai_src = ADD_N;
            bi_src = ADD_BUS;
            ci = 0;
            alu_OP = ALU_ADD;
        end else if (inc_ai) begin
            bi_src = ADD_Z;
            ai_src = ADD_BUS;
            ci = 1;
            alu_OP = ALU_ADD;
        end else if (dec_ai) begin
            bi_src = ADD_N;
            ai_src = ADD_BUS;
            ci = 0;
            alu_OP = ALU_ADD;
        end

    end


    `ifdef DEBUG_CPU
        `include "debug/debug.sv"
    `endif 
    `ifdef DEBUG_REG
        always_comb begin
            pc_dbg = pc;
            s_dbg = s;
            a_dbg = a;
            x_dbg = x;
            y_dbg = y;
            p_dbg = p;
        end
        int cycle;
        always_ff @(posedge clk_m1) cycle <= reg_set_en ? 0 : cycle+1;    
    `endif 


endmodule

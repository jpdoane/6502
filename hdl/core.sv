`include "defs.svi"


module core #(
    parameter BOOT_ADDR=16'h0)
    (
    input  logic i_clk, i_rst,
    input  logic [7:0] i_data,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,

    output logic [15:0] addr,
    output logic [7:0] dor,
    output logic RW
    );

    // busses
    // db: data bus: typically memory read, but can be addr_lo or zero
    // sb: secondry bus: read from registers (a,x,y,s, zero)
    // res: result bus: update register (a,x,y,s,d [mem write], z[disabled])
    logic [7:0] db, sb, res;

    // registers
    logic [15:0] pc, pcsel, nextpc;

    // pc logic
    logic jump, pc_inc;
    assign pcsel = jump ? addr : pc;
    assign nextpc = (pc_inc || jump) ? pcsel + 1 : pcsel;

    logic [7:0] a, s, x, y, p;
    logic [7:0] radl, radh;
    logic [7:0] ir;
    logic [7:0] add;
    logic IPC;

    logic [2:0] res_dst;

    always @(posedge i_clk ) begin
        if (i_rst) begin
            a <= 0;
            x <= 0;
            y <= 0;
            s <= 0;
            radh <= 0;
            radl <= 0;
            pc <= BOOT_ADDR;
            //p updated elsewhere...
        end else begin
            a <= a;
            x <= x;
            y <= y;
            s <= s;

            radh <= addr[15:8];
            radl <= addr[7:0];
            pc <= nextpc;

            case(res_dst)
                REG_A: a <= res;
                REG_X: x <= res;
                REG_Y: y <= res;
                REG_S: s <= res;
                REG_P: p <= res & 8'hef; // clear break flag on p read
                default: begin end
            endcase
        end
    end

    assign RW = !(res_dst == REG_DATA);
    assign dor = RW ? 0 : res;

    // bus source select
    logic [2:0] db_src;
    logic [2:0] sb_src;
    logic [1:0] res_src;
    always @(*) begin
        case(db_src)
            DB_DATA:    db = i_data;
            DB_PCL:     db = pc[7:0];
            DB_PCH:     db = pc[15:8];
            DB_S:       db = s;
            DB_P:       db = p;
            DB_A:       db = a;
            default:    db = 8'h0;
        endcase
        case(sb_src)
            REG_A: sb = a;
            REG_X: sb = x;
            REG_Y: sb = y;
            REG_S: sb = s;
            REG_P: sb = p | 8'h10; // set break flag on p write
            REG_ADD: sb = add;
            REG_DATA: sb = i_data;
            default: sb = 8'h0;
        endcase
        case(res_src)
            RES_DB: res = db;
            RES_SB: res = sb;
            RES_ADD: res = add;
            default: res = 8'h0;
        endcase
    end

    // alu source select
    logic [1:0] ai_src, bi_src;
    logic [7:0] ai, bi;
    always @(*) begin
        case(ai_src)
            ADD_BUS:    ai = sb;
            ADD_INVBUS: ai = ~sb;
            ADD_N:      ai = 8'hff;
            default:    ai = 8'h0;
        endcase
        case(bi_src)
            ADD_BUS:    bi = db;
            ADD_INVBUS: bi = ~db;
            ADD_N:      bi = 8'hff;
            default:    bi = 8'h0;
        endcase
    end

    //alu
    logic [7:0] aorb, alu_out;
    logic ci;
    logic aluN, aluZ, aluC, aluV;

    logic [2:0] alu_OP;
    logic [1:0] alu_carry_in; // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]
    logic [7:0] status, rstatus, status_mask;

    always @(*) begin
        aorb = ai | bi;
        ci = alu_carry_in[1] ? p[0] : alu_carry_in[0];

        alu_out = 0;
        aluC = 0;

        case(alu_OP)
            // verilator lint_off WIDTH
            ALU_ADD:    {aluC, alu_out} = ai + bi + ci;
            // verilator lint_on WIDTH
            ALU_BIT,
            ALU_AND:    alu_out = ai & bi;
            ALU_OR:     alu_out = aorb;
            ALU_XOR:    alu_out = ai ^ bi;
            // unary shifts operate on ai|bi, so unused port must be zeroed
            ALU_SR:     {alu_out, aluC} = {ci, aorb};
            ALU_SL:     {aluC, alu_out} = {aorb, ci};
            default:    begin end
        endcase

        aluZ = ~|alu_out;

        if(alu_OP == ALU_BIT) begin
            aluN = bi[7];
            aluV = bi[6];
        end else begin
            aluN = alu_out[7];
            aluV = ai[7] ^ bi[7] ^ aluC ^ aluN;
        end
            
        status = {aluN, aluV, 4'b0, aluZ, aluC};
    end

    // register alu and p
    logic[7:0] set_mask, clear_mask;
    always @(posedge i_clk ) begin
        if (i_rst) begin
            add <= 8'b0;
            p <= 8'b0;
            rstatus <= 8'h0;
        end else begin
            add <= alu_out;
            rstatus <= status;
            p <= ~clear_mask & (set_mask | (status_mask & status) | (~status_mask & p));
        end
    end


    // state
    logic [4:0] state, initial_state;
    logic state_skip;

    // ir fetch
    // assign SYNC = state == T0_FETCH;
    always @(posedge i_clk ) begin
        if (i_rst)          ir <= 8'hea; //NOP
        else if (state==T1_DECODE) ir <= i_data;
    end
    logic [7:0] current_op;
    assign current_op = (state==T1_DECODE) ? i_data : ir;

    // decode opcode
    // rather than a huge mux on full opcode, utilize layout patterns
    // https://www.masswerk.at/6502/6502_instruction_set.html#layout
    logic [2:0] op_a;
    logic [2:0] op_b;
    logic [1:0] op_c;
    assign {op_a, op_b, op_c} = current_op;

    // branch logic
    logic take_branch;
    always @(*) begin
        case(op_a)
            3'h0:   take_branch = !p[7]; // BPL
            3'h1:   take_branch = p[7]; // BMI
            3'h2:   take_branch = !p[6]; // BVC
            3'h3:   take_branch = p[6]; // BVS
            3'h4:   take_branch = !p[0]; // BCC
            3'h5:   take_branch = p[0]; // BCS
            3'h6:   take_branch = !p[1]; // BNE
            3'h7:   take_branch = p[1]; // BEQ
        endcase
    end

    // decode address mode, which determines initialtake_branch state
    // primarily differentiated by op_b
    logic op_SB;
    logic idx_XY;           // select index X(1) or Y(0) for abs,X/Y and zpg,X/Y ops
    always @(*) begin
        op_SB = 0;
        idx_XY = 1;
        initial_state = T_JAM;
        case(op_b)
            3'h0:   if (op_c[0] == 1) initial_state = T2_XIND;           // X,ind
                    else if (op_a[2] == 0) initial_state = T_JAM;        // BRK, JSR, RTI, RTS
                    else initial_state = T0_FETCH;                             // LD/CP imm
            3'h1:   initial_state = T2_ZPG;                              // all zpg
            3'h2:   begin
                        initial_state = (op_c == 0 && !op_a[2]) ? T2_STACK : // c=0, a<4 stack ops
                                                                 T0_FETCH ;  // others imm or impl 
                        op_SB = !op_c[0];                                     // c=0,2 are single byte
                    end
            3'h3:   if (op_a[2:1] == 'b01 && op_c == 0) initial_state = T2_JMP;
                    else if (op_a[2:1] == 'b01 && op_c == 'b00) initial_state = T_JAM;      // jump abs,ind
                    else initial_state = T2_ABS;                         // abs ops
            3'h4:   if (op_c == 'b00)
                        initial_state =  take_branch ? T2_BRANCH : T0_FETCH; // branches
                    else begin
                        idx_XY = 0;
                        initial_state = T2_INDY;                        // alu ind,Y ops
                    end
            3'h5:   begin
                        initial_state = T2_ZPGXY;                        // zpg,X/Y
                        idx_XY = !(op_c==2 && op_a[2:1]=='b10);
                    end
            3'h6:   if (op_c[0] == 0) begin
                        op_SB = 1;
                        initial_state = T0_FETCH;               // impl
                    end 
                    else begin
                        initial_state = T2_ABSXY;                        // abs,Y
                        idx_XY = 0;
                    end
            3'h7:   begin
                        initial_state = T2_ABSXY;                        // abs,X/Y
                        idx_XY = !(op_c==2 && op_a[2:1]=='b10);
                    end
        endcase
    end

    // decode opcode type and data access pattern
    // primarily differentiated by op_a and op_c
    logic [2:0] ex_sb_src;   // sb bus source (registers)
    logic [2:0] ex_db_src;   // db bus source (data/addr)
    logic [1:0] ex_res_src;  // source of result (alu, sb, db)
    logic [2:0] ex_res_dst;  // location to store result (registers, mem)

    logic [2:0] ex_alu_OP;
    logic [1:0] ex_ai_src;   // alu ai src
    logic [1:0] ex_bi_src;   // alu bi src
    logic [1:0] ex_carry_in; // 00: Cin=0, 01: Cin=1, 10: Cin=P[C]
    logic [7:0] ex_status_mask;  // mask to update status flags: NZCIDV
    logic stack_push, stack_pull;

    always @(posedge i_clk ) begin
        //default alu behavor: A + mem -> A
        ex_db_src <= DB_DATA;
        ex_sb_src <= REG_A;
        ex_ai_src <= ADD_BUS;
        ex_bi_src <= ADD_BUS;
        ex_res_src <= RES_ADD;
        ex_res_dst <= REG_A;

        ex_alu_OP <= ALU_ADD;
        ex_status_mask <= 0;        // default do not update status flag
        ex_carry_in <= 2'b00;   // default carry in <= 0

        set_mask <= 8'h0;
        clear_mask <= 8'h0;

        case(op_c)
            0: begin
                //primarily control flow, special, and load/store/tranfer
                ex_res_dst <= REG_Z; // default no register update

                if(op_b==4) begin end// branch op
                else if(op_b==6) begin   // set/clear flags
                    case(op_a)
                        0: clear_mask[0] <= 1; // CLC
                        1: set_mask[0] <= 1;   // SEC
                        2: clear_mask[2] <= 1; // CLI
                        3: set_mask[2] <= 1; // SEI
                        4:  begin //TYA
                            ex_sb_src <= REG_Y;
                            ex_res_src <= RES_SB;
                            ex_res_dst <= REG_A;
                            end
                        5: clear_mask[6] <= 1; // CLV
                        6: clear_mask[3] <= 1; // CLD
                        7: set_mask[3] <= 1; // SED
                    endcase
                end
                else if(op_a[2] == 0) begin // a=0:3, b != 6
                    if (op_b == 3'h2) begin //Stack push/pull
                        ex_res_src <= RES_DB;
                        if (op_a[0]) begin
                            // pull
                            ex_res_src <= RES_DB;
                            ex_res_dst <= op_a[1] ? REG_A : REG_P;
                        end else begin
                            // push
                            ex_db_src <= op_a[1] ? DB_A : DB_P;
                            ex_res_dst <= REG_DATA;
                        end
                    end if (op_a[1:0] == 2'b01 && op_b[0]) begin// BIT
                        // ex_sb_src <= REG_A;
                        // ex_db_src <= DB_DATA;
                        ex_alu_OP <= ALU_BIT;
                        ex_res_dst <= REG_Z;
                        ex_status_mask <= 8'b11000010;
                    end
                    // control flow and special ops
                    // TODO....
                end
                else if(op_b == 3'b010) begin // a=4:7, b=2
                    //DEY, TAY, INY, INX
                    if (op_a == 5) begin //TAY
                        ex_res_dst <= REG_Y;
                        ex_sb_src <= REG_A;
                        ex_res_src <= RES_SB;
                    end else begin // DEY, INY, INX
                        ex_sb_src <= REG_Y;
                        ex_bi_src <= ADD_Z;
                        ex_res_src <= RES_ADD;
                        ex_res_dst <= REG_Y;
                        ex_status_mask <= 8'b10000010; // status mask for inc/dec
                        if(op_a==4) begin //DEY
                            ex_bi_src <= ADD_N; //bi <= -1
                            ex_carry_in <= 2'b00; // C=0 
                        end else if(op_a==6) begin //INY
                            ex_carry_in <= 2'b01; // C=1
                        end else if(op_a==7) begin //INX
                            ex_sb_src <= REG_X;
                            ex_res_dst <= REG_X;
                            ex_carry_in <= 2'b01; // C=1
                        end
                    end
                end
                // remaining options are restriced to a>=4, b=0,1,3,5,7
                else if(op_a==4) begin// STY
                    ex_sb_src <= REG_Y;
                    ex_res_src <= RES_SB;
                    ex_res_dst <= REG_DATA;
                end
                else if(op_a == 5) begin //LDY
                    ex_res_src <= RES_DB;
                    ex_res_dst <= REG_Y;
                end
                else begin // op_a=6-7: CPX/CPY
                    // ex_alu_OP <= ALU_ADD;    //already default
                    ex_carry_in <= 2'b01; // C=1 (equivalent to borrow=0)
                    ex_bi_src <= ADD_INVBUS; // ai-bi
                    ex_sb_src <= op_a[0] ? REG_X : REG_Y;
                    ex_res_dst <= REG_Z; //dont store result, just status mask
                    ex_status_mask <= 8'b10000011;
                end
            end
        1:  begin
                // accumulator operations
                // already default:
                // ex_sb_src <= REG_A;
                // ex_db_src <= DB_DATA;
                // ex_res_src <= RES_ADD;
                // ex_res_dst <= REG_A;
                case(op_a)
                    0:  begin   //ORA
                        ex_alu_OP <= ALU_OR;
                        // ex_carry_in <= 2'b00;
                        ex_status_mask <= 8'b10000010;
                        end
                    1:  begin   //AND
                        ex_alu_OP <= ALU_AND;
                        // ex_carry_in <= 2'b00;
                        ex_status_mask <= 8'b10000010;
                        end
                    2:  begin   //EOR
                        ex_alu_OP <= ALU_XOR;
                        // ex_carry_in <= 2'b00;
                        ex_status_mask <= 8'b10000010;
                        end
                    3:  begin   //ADC
                        // ex_alu_OP <= ALU_ADD;
                        ex_carry_in <= 2'b10;   // C=P[C]
                        ex_status_mask <= 8'b11000011;
                        end
                    4:  begin //STA
                        // ex_sb_src <= REG_A;
                        ex_res_src <= RES_SB;
                        ex_res_dst <= REG_DATA;
                        end
                    5:  begin //LDA
                        ex_res_src <= RES_DB;
                        end
                    6:  begin // CMP;
                        // ex_alu_OP <= ALU_ADD;
                        ex_carry_in <= 2'b01; // C=1 (borrow=0)
                        ex_bi_src <= ADD_INVBUS;        // negate DB
                        ex_status_mask <= 8'b10000011;
                        ex_res_dst <= REG_Z; //dont update registers
                        end
                    7:  begin // SBC;
                        // ex_alu_OP <= ALU_ADD;
                        ex_carry_in <= 2'b10; // C=P[C]
                        ex_bi_src <= ADD_INVBUS;        // negate DB
                        ex_status_mask <= 8'b11000011;
                        end
                endcase
            end
        2:  begin
                // mostly unary operations (shift, rot, inc, dec)
                if (op_b == 2 || op_b == 4) begin
                    // defaults:
                    // ex_sb_src <= REG_A;
                    // ex_res_src <= RES_ADD;
                    // ex_res_dst <= REG_A;
                    ex_bi_src <= ADD_Z; // op(reg,0)
                end else begin
                    // defaults
                    // ex_db_src <= DB_DATA;
                    // ex_res_src <= RES_ADD;
                    ex_ai_src <= ADD_Z; // op(0,MEM)
                    ex_res_dst <= REG_DATA;
                end
                
                case(op_a)
                    0:  begin // ASL
                        ex_alu_OP <= ALU_SL;
                        // ex_carry_in <= 2'b00;    //already default
                        ex_status_mask <= 8'b10000011;
                        end
                    1:  begin // ROL
                        ex_alu_OP <= ALU_SL;
                        ex_carry_in <= 2'b10;       // C <= P[C]
                        ex_status_mask <=8'b10000011;
                        end
                    2:  begin // LSR
                        ex_alu_OP <= ALU_SR;
                        // ex_carry_in <= 2'b00;
                        ex_status_mask <= 8'b00000011;
                        end
                    3:  begin // ROR
                        ex_alu_OP <= ALU_SR;
                        ex_carry_in <= 2'b10; // C <= P[C]
                        ex_status_mask <= 8'b10000011;
                        end
                    4:  begin // copy from X: STX, TXA, TXY
                        ex_sb_src <= REG_X;
                        ex_res_src <= RES_SB; 
                        case(op_b)
                            2:          ex_res_dst <= REG_A;
                            6:          ex_res_dst <= REG_S;
                            default:    ex_res_dst <= REG_DATA;
                        endcase
                        end
                    5:  begin // copy to X: LDX, TAX, TSX
                        ex_res_dst <= REG_X;
                        ex_res_src <= (op_b == 2 || op_b == 6) ? RES_SB : RES_DB; 
                        ex_sb_src <= (op_b == 6) ? REG_S : REG_A;
                        end
                    6:  begin // DEC
                        // ex_alu_OP <= ALU_ADD;            //already default
                        // ex_carry_in <= 2'b00;        //already default
                        if (op_b == 2)  begin
                                        //DEX
                                        ex_sb_src <= REG_X;
                                        ex_res_dst <= REG_X; 
                                        ex_bi_src <= ADD_N; // op(reg,-1)
                                        end
                        else            ex_ai_src <= ADD_N;  // op(-1,MEM)
                        ex_status_mask <= 8'b10000010;
                        end
                    7:  begin // INC, NOP
                        // ex_alu_OP <= ALU_ADD;
                        ex_carry_in <= 2'b01; // C=1
                        ex_status_mask <= 8'b10000010;
                        if (op_b==2) begin
                            ex_status_mask <= 0;
                            ex_res_dst <= REG_Z;
                        end
                        end
                endcase
            end
        endcase
    end


    // memory modes
    // determined by: ex_res_src and ex_res_dst
    logic rd_op, ld_op, st_op, rmw_op;
    always @(*) begin
        rd_op = 0;
        ld_op = 0; //also stack pull
        st_op = 0; //also stack push
        rmw_op = 0;
        if(ex_res_dst == REG_DATA)
            if(ex_res_src == RES_ADD) rmw_op = 1;
            else st_op = 1;                   
        else
            if(ex_res_src == RES_DB) ld_op = 1;
            else rd_op = 1;
    end    

    //state machine
    logic jump_ind;
    always @(posedge i_clk ) begin
        jump_ind <= 0;
        if (i_rst) begin
            state <= T_BOOT;
        end
        else case(state)
            T0_FETCH:       state <= T1_DECODE;
            T1_DECODE:       state <= initial_state;
            T2_ZPG:   state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_ZPGXY: state <= T3_ZPGXY;
            T3_ZPGXY: state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_ABS:   state <= T3_ABS;
            T3_ABS:   state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_ABSXY: state <= T3_ABSXY;
            T3_ABSXY: state <= !state_skip ? T4_ABSXY : rmw_op ? T_RMW_EXEC : T0_FETCH;
            T4_ABSXY: state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_XIND:  state <= T3_XIND;
            T3_XIND:  state <= T4_XIND;
            T4_XIND:  state <= T5_XIND;
            T5_XIND:  state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T2_INDY:  state <= T3_INDY;
            T3_INDY:  state <= T4_INDY;
            T4_INDY:  state <= !state_skip ? T5_INDY : rmw_op ? T_RMW_EXEC : T0_FETCH;
            T5_INDY:  state <= rmw_op ? T_RMW_EXEC : T0_FETCH;
            T_RMW_EXEC:  state <= T_RMW_STORE;
            T_RMW_STORE: state <= T0_FETCH;

            T2_JMP:   begin
                        // register whether we are on 1st step of indirect jump
                        jump_ind <= ir[5] & !jump_ind;
                        state <= T3_JMP;
                      end 
            // fetching new address
            // if abs jump, this is new opcode, go to decode
            // if ind jump, this is new pc, redo jump
            T3_JMP:   begin
                        state <= jump_ind ? T2_JMP : T1_DECODE;
                        jump_ind <= jump_ind;
                      end

            T2_BRANCH:   state <= T3_BRANCH;
            T3_BRANCH:   state <= state_skip ? T1_DECODE : T4_BRANCH;
            T4_BRANCH:   state <= T1_DECODE;

            T2_STACK:    state <= T3_STACK;
            T3_STACK:    state <= st_op ? T1_DECODE : T0_FETCH;
            
            T_BOOT:   state <= T0_FETCH;
            default:  state <= T_JAM;
        endcase
    end

    // addr control
    logic [7:0] zeropage = 8'h00;
    logic [7:0] stackpage = 8'h01;
    logic compute_result, save_result;

    always @(*) begin
        addr = pc;

        state_skip = 0;
        compute_result = 0;
        save_result = 0;
        jump = 0;
    
        // default alu when not processing result:
        // result = data + idx_XY
        // cin = 0, st_mask = 0, do not store result
        ai_src = ADD_BUS;
        bi_src = ADD_BUS;
        db_src = DB_DATA;
        sb_src = idx_XY ? REG_X : REG_Y;
        alu_OP = ALU_ADD;
        alu_carry_in = 0;
        status_mask = 0;
        res_src = RES_ADD;
        res_dst = REG_Z;

        jump = 0;
        pc_inc = 0;

        case(state)
            T0_FETCH:         begin
                        pc_inc = 1;             // next pc = pc++
                        compute_result = rd_op; // compute read op
                        save_result = ld_op;    // save load op
                        end

            T1_DECODE:  begin
                        // fetch pc, next pc = pc++ unless new opcode is only 1 byte
                        pc_inc = !op_SB;
                        save_result = rd_op;    // store result of read op
                        end

            T2_ZPG:     begin
                        addr = {zeropage, db};  // fetch {0,BAL}
                        save_result = st_op;
                        end

            T2_ZPGXY:   addr = {radh,radl};     // hold addr, compute LL = BAL + X/Y
            T3_ZPGXY:   begin
                        addr = {zeropage,res};  // fetch {0,LL}
                        save_result = st_op;
                        end

            T2_JMP,
            T2_ABS:     begin
                        sb_src = REG_Z;         // alu nop: hold LL in add
                        // fetch BAH
                        pc_inc = 1;             // next pc = pc++
                        end
            T3_ABS:     begin
                        addr = {db,res};        // fetch {BAH,BAL}
                        save_result = st_op;
                        end

            T2_ABSXY:   begin
                        // compute LL = BAL + X/Y
                        //fetch BAH
                        pc_inc = 1;                // fetch pc, pc++
                        end                
            T3_ABSXY:   begin
                        addr = {db,res};        // fetch {BAH,LL}, assuming no carry
                        if (rstatus[0]) begin //carry
                            sb_src = REG_Z;       // inc BAH
                            alu_carry_in = 2'b01;
                        end else begin
                            state_skip = 1;
                            save_result = st_op;
                        end
                        end
            T4_ABSXY:   begin
                        addr = {res,radl};      // fetch {HH,LL}
                        save_result = st_op;
                        end

            T2_XIND:    addr = {zeropage, db};  // fetch {0,BAL} (discarded), compute BAL+X
            T3_XIND:    begin
                        addr = {zeropage, res}; // fetch LL = {0,BAL+X}
                        db_src = DB_Z;         // compute BAL+X+1;
                        sb_src = REG_ADD;
                        alu_carry_in = 2'b01;
                        end
            T4_XIND:    begin
                        sb_src = REG_Z;         // hold LL in add
                        addr = {zeropage, res}; // fetch {0,BAL+X+1}
                        end
            T5_XIND:    begin
                        addr = {db, res};       // fetch {HH,LL}
                        save_result = st_op;
                        end

            T2_INDY:    begin
                        addr = {zeropage, db};  // fetch BAL = {00,IAL}
                        sb_src = REG_Z;         // compute IAL + 1
                        alu_carry_in = 2'b01;
                        end
            T3_INDY:    addr = {zeropage, res}; // fetch BAH = {00,IAL+1}, compute BAL + Y
            T4_INDY:    begin
                        addr = {db, res};       // fetch {BAH,BAL+Y}, assuming no carry
                        if (rstatus[0]) begin
                            sb_src = REG_Z;         // inc BAH
                            alu_carry_in = 2'b01;
                        end else begin
                            state_skip = 1;
                            save_result = st_op;
                        end
                        end
            T5_INDY:    begin
                        addr = {res, radl};     // fetch {BAH+1,BAL+Y}
                        save_result = st_op;
                        end
            T_RMW_EXEC: begin
                        addr = {radh,radl};     // hold addr
                        compute_result = 1;
                        end
            T_RMW_STORE:begin
                        addr = {radh,radl};     // hold addr
                        save_result = 1;
                        end

            T2_BRANCH:   begin
                        pc_inc = 1;             // next pc = pc++
                        // pcl = offset + pcl
                        db_src = DB_PCL;        
                        sb_src = REG_DATA;
                        end

            T3_BRANCH:  begin
                        addr = {radh,add};     // {pch, offset + pcl}
                        jump = 1;              // addr -> pc
                        if (rstatus[0] && !rstatus[7]) begin
                             // C=1, N=0: jumped to higher page, pch must be incremented
                            db_src = DB_PCH;         // inc PCH
                            ai_src = ADD_Z;
                            alu_carry_in = 2'b01;
                        end else if (rstatus[6] && rstatus[7]) begin
                             // V=1, N=1: jumped to lower page, pch must be decremented
                            db_src = DB_PCH;         // dec PCH
                            ai_src = ADD_N;
                        end else begin
                            // pch is valid, this will be valid T0_FETCH fetch
                            // nextpc = pc++
                            pc_inc = 1;         
                            state_skip = 1;
                        end
                        end

            T4_BRANCH: begin
                        addr = {add,radl};     // {pch, offset + pcl}
                        jump = 1;              // addr -> pc
                        pc_inc = 1;            // nextpc = pc++
                        end

            T3_JMP:     begin
                        addr = {db,add};        // fetch new pc
                        jump = 1;               // update pc
                        end


            // stack data movement is on db (ptr arithmetic on sb)
            // to push:
            // ex_db_src: what to push
            // ex_res_src: RES_DB
            // ex_res_dst: REG_DATA
            // to pull:
            // ex_db_src: DB_DATA
            // ex_res_src: RES_DB
            // ex_res_dst: what to pull
            T2_STACK:    begin
                        addr = {stackpage,s};
                        if (st_op) begin
                            //push to stack
                            db_src = ex_db_src;
                            res_src = RES_DB;
                            res_dst = REG_DATA;
                        end
                        // inc/dec s
                        sb_src = REG_S;
                        bi_src = st_op ? ADD_N : ADD_Z;    // push: s--
                        alu_carry_in = ld_op ? 'b01 : 0;   // pull: s++
                        end

            T3_STACK:   begin
                        // update stack register
                        res_src = REG_ADD;
                        res_dst = REG_S;
                        
                        // for push, this is T0 and we need to fetch next addr
                        pc_inc = st_op;             // next pc = pc++

                        // for pull, (s++) loaded to ex_res_dst during next T0
                        if(ld_op) addr = {stackpage,add};
                        end


            default:    begin end
        endcase

        // compute and save result
        if (compute_result || save_result) begin
            db_src = ex_db_src;
            sb_src = ex_sb_src;
        end
        if (compute_result) begin
            alu_OP = ex_alu_OP;
            ai_src = ex_ai_src;
            bi_src = ex_bi_src;
            alu_carry_in = ex_carry_in;
            status_mask = ex_status_mask;
        end
        if (save_result) begin
            res_src = ex_res_src;
            res_dst = ex_res_dst;
        end

    end


endmodule

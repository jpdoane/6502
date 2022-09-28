`include "defs.svi"

module control(
    input  logic i_clk, i_rst,
    input  logic [7:0]	din,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,
    input  logic alu_carry,

    output logic [7:0] ir,
    output logic SYNC,
    output st_ctl ctl
    );

    e_state state, decode_state;
    logic single_byte, skip_state;
    e_index idx_XYZ; // index pulled from X,Y,or Z;

    logic execute;
    logic inc_alu;
    assign execute = state == T0;

    // ir fetch
    assign SYNC = state == T0;
    always @(posedge i_clk ) begin
        if (i_rst)          ir <= 8'hea; //NOP
        else if (state==T1) ir <= din;
    end
    logic [7:0] current_op;
    assign current_op = (state==T1) ? din : ir;

    // decode opcode
    // rather than a huge mux on full opcode, utilize layout patterns
    // https://www.masswerk.at/6502/6502_instruction_set.html#layout
    wire [2:0] op_a;
    wire [2:0] op_b;
    wire [1:0] op_c;
    assign {op_a, op_b, op_c} = current_op;
    always @(*) begin
        idx_XYZ = IZ;
        single_byte = 0;
        case(op_b)
            3'h0:  if (op_c == 2'b?1) begin
                    decode_state = T2_XIND;            // X,ind
                    idx_XYZ = IX;
                end
                else
                    if (op_a == 3'b000) begin
                    // if (op_a == 3'b0??) begin
                        decode_state = UNIMPLEMENTED;  // BRK, JSR, RTI, RTS
                        single_byte = (op_a != 3'b001);// all but JSR is single byte
                    end
                    else decode_state = T0;            // LD/CP imm
            3'h1:  decode_state = T2_ZPG;                 // all zpg
            3'h2:  begin
                    decode_state = T0;                 // all imm or impl
                    single_byte = (op_c == 2'b?0);     // various impl and accum ops
                end
            3'h3:  if (op_a == 3'b01? && op_c == 3'b00)
                    decode_state = UNIMPLEMENTED;      // jump abs,ind
                else decode_state = T2_ABS;            // abs ops
            3'h4:  if (op_c == 2'b00)
                    decode_state = UNIMPLEMENTED;            // branches
                else begin
                    decode_state = T2_INDY;            // alu ind,Y ops
                    idx_XYZ = IY;                       // set index Y
                end
            3'h5:  begin
                    decode_state = T2_ZPGXY;            // zpg,X/Y
                    if (op_c == 2'b1? && op_a == 3'b10?)
                        idx_XYZ = IY;
                    else
                        idx_XYZ = IX;
                end
            3'h6:  if (op_c == 2'b?0) begin
                    decode_state = T0;                 // impl
                    single_byte = 1;
                end
                else begin
                    decode_state = T2_ABS;           // abs,Y ops
                    idx_XYZ = IY;
                end
            3'h7:  begin
                    decode_state = T2_ABS;           // abs,X/Y
                    if (op_c == 2'b1? && op_a == 3'b10?)
                        idx_XYZ = IY;
                    else
                        idx_XYZ = IX;
                end
            default: decode_state = UNIMPLEMENTED;
        endcase
    end

    integer cycle = 0;
    always @(posedge i_clk ) begin
        if (i_rst) cycle <= 0;
        else if (state==BOOT) cycle <= 0;
        else cycle <= cycle+1;
    end

    //state machine
    always @(posedge i_clk ) begin
        if (i_rst) state <= BOOT;
        else case(state)
            T0:       state <= T1;
            T1:       state <= decode_state;
            T2_ZPG:   state <= T0;
            T2_ZPGXY: state <= T3_ZPGXY;
            T3_ZPGXY: state <= T0;
            T2_ABS:   state <= T3_ABS;
            T3_ABS:   state <= skip_state ? T0 : T4_ABS;
            T4_ABS:   state <= T0;
            T2_XIND:  state <= T3_XIND;
            T3_XIND:  state <= T4_XIND;
            T4_XIND:  state <= T5_XIND;
            T5_XIND:  state <= T0;
            T2_INDY:  state <= T3_INDY;
            T3_INDY:  state <= T4_INDY;
            T4_INDY:  state <= skip_state ? T0 : T5_INDY;
            T5_INDY:  state <= T0;
            BOOT:     state <= T0;
            default:  state <= UNIMPLEMENTED;
        endcase
    end


    logic op_or, op_and, op_xor, op_add, op_sub, op_cmp;
    logic op_asl, op_rol, op_lsr, op_ror, op_dec, op_inc;
    //decode alu operation
    always @(*) begin
        op_or = op_c[0] && op_a == 0;
        op_and = op_c[0] && op_a == 1;
        op_xor = op_c[0] && op_a == 2;
        op_add = op_c[0] && op_a == 3;
        op_cmp = ( op_c[0] && op_a == 6 ) ||
                 ( current_op == 8'b11?_0?1_00 || current_op == 8'b11?_010_00 );

        op_sub  = (op_c[0] && (op_a == 7)) || op_cmp;

        op_asl = op_c[1] && (op_a == 0);
        op_rol = op_c[1] && (op_a == 1);
        op_lsr = op_c[1] && (op_a == 2);
        op_ror = op_c[1] && (op_a == 3);
        op_dec = op_c[1] && (op_a == 6);
        op_inc = (op_c[1] && (op_a == 7) && !(op_b==2) ) ||
                 ((op_c == 0) && (op_a[2:1] == 2'b11) && (op_b==2) );


        // always add when not in execute stage
        ctl.SUMS = (op_add || op_sub) || !execute;
        ctl.ANDS = execute && op_and;
        ctl.ORS = execute && op_or;
        ctl.EORS = execute && op_xor;
        ctl.SRS = 0;
        // .todo

        ctl.DB0C = 0;
        ctl.DB1Z = 0;
        ctl.DBZZ = 0;
        ctl.DB2I = 0;
        ctl.DB3D = 0;
        ctl.DB6V = 0;
        ctl.DB7N = 0;

        ctl.IR5C = 0;
        ctl.IR5V = 0;
        ctl.IR5D = 0;
        ctl.IR5I = 0;

        ctl.ACRC = 0;
        ctl.AVRV = 0;
        ctl.IV = 0;

        if (execute && (op_add || op_sub)) begin
            ctl.ACRC = 1;
            ctl.DBZZ = 1;
        end
        //???

        ctl.IADDC = inc_alu;
    end

    always @(*) begin

        ctl.DLDB = 0;
        ctl.PCLDB = 0;
        ctl.SBDB = 0;
        ctl.ACDB = 0;
        ctl.PDB = 0;
        ctl.PCLDB = 0;
        ctl.PCHDB = 0;

        ctl.XSB = 0;
        ctl.YSB = 0;
        ctl.DBSB = 0;
        ctl.ADHSB = 0;
        ctl.ADDSB0_6 = 0;
        ctl.ADDSB7 = 0;
        ctl.SSB = 0;
        ctl.ACSB = 0;

        // internal address bus
        ctl.PCLADL = 0;
        ctl.PCHADH = 0;
        ctl.DLADL = 0;
        ctl.ADDADL = 0;
        ctl.SADL = 0;
        ctl.ZADL0 = 0;
        ctl.ZADL1 = 0;
        ctl.ZADL2 = 0;
        ctl.ZADH1_7 = 0;
        ctl.ZADH0 = 0;
        ctl.DLADH = 0;
        ctl.SBADH = 0;

        // external address update (default enable)
        ctl.ADLABL = state != BOOT;
        ctl.ADHABH = state != BOOT;

        // alu sources
        ctl.ADLADD = 0;
        ctl.INVDBADD = 0;
        ctl.DBADD = 0;
        ctl.SBADD = 0;
        ctl.ZADD = 0;
        inc_alu = 0;

        //register store
        ctl.SBAC = 0;
        ctl.SBX = 0;
        ctl.SBY = 0;
        ctl.SBS = 0;
        ctl.SS = 0;

        // pc source
        ctl.ADLPCL = 0;
        ctl.ADHPCH = 0;
        ctl.IPC = 0;

        skip_state = 0;
        
        ctl.RW = 1;
        ctl.DAA = 0;
        ctl.DSA = 0;

        case(state)
            T0:     begin
                    // exectution of prev OPCODE
                    // for now, lets assume that we are performing alu op on db & acc
                    ctl.DLDB = 1;
                    ctl.ACSB = 1;
                    ctl.DBADD = 1;
                    ctl.SBADD = 1;

                    // fetch next op @ pc
                    ctl.PCLADL = 1;
                    ctl.PCHADH = 1;
                    // pc++
                    ctl.IPC = 1;
                    end
            T1:     begin
                    // storage of prev OPCODE
                    // for now, lets assume that we putting the alu result back to acc
                    ctl.ADDSB7 = 1;
                    ctl.ADDSB0_6 = 1;
                    ctl.SBAC = ir != 8'hea; // not NOP

                    // fetch data @ pc+1
                    ctl.PCLADL = 1;
                    ctl.PCHADH = 1;
                    // pc++ unless 1 byte opcode
                    ctl.IPC = !single_byte;       
                    end
            T2_ZPG: begin
                    // data->adl, adh=0 (fetch LL00)
                    ctl.DLADL = 1;
                    ctl.ZADH1_7 = 1;
                    ctl.ZADH0 = 1;
                    end
            T2_ZPGXY:begin
                    // no fetch, hold external bus
                    ctl.ADLABL = 0;
                    ctl.ADHABH = 0;
                    // compute LL = data + X/Y
                    ctl.DLDB = 1;
                    ctl.DBADD = 1;
                    ctl.XSB = idx_XYZ == IX;
                    ctl.YSB = idx_XYZ == IY;
                    ctl.SBADD = 1;
                    end
            T3_ZPGXY:begin
                    // LL -> adl, adh=0 (fetch LL00)
                    ctl.ADDADL = 1;
                    ctl.ZADH1_7 = 1;
                    ctl.ZADH0 = 1;
                    end
            T2_ABS: begin
                    // compute LL = data + X/Y/Z
                    ctl.DLDB = 1;
                    ctl.DBADD = 1;
                    ctl.XSB = idx_XYZ == IX;
                    ctl.YSB = idx_XYZ == IY;
                    if (idx_XYZ == IZ) ctl.ZADD = 1;
                    else ctl.SBADD = 1;
                    // fetch HH
                    ctl.PCLADL = 1;
                    ctl.PCHADH = 1;
                    // pc++
                    ctl.IPC = 1;
                    end
            T3_ABS: begin
                    // fetch [LLHH], assuming no carry
                    ctl.ADDADL = 1;
                    ctl.DLADH = 1;
                    // if no carry, skip T4_ABS
                    skip_state = !alu_carry;
                    // in case there was a carry, compute HH++
                    ctl.DLDB = 1;
                    ctl.DBADD = 1;
                    ctl.ZADD = 1;
                    inc_alu = 1;
                    end
            T4_ABS: begin
                    // fetch [LLHH] with updated HH++ due to carry
                    //hold ABL
                    ctl.ADLABL = 0;
                    // HH++ -> sb -> adh -> abh
                    ctl.ADDSB7 = 1;
                    ctl.ADDSB0_6 = 1;
                    ctl.SBADH = 1;
                    end
            T2_XIND:begin
                    // no address fetch, hold external AB
                    ctl.ADLABL = 0;
                    ctl.ADHABH = 0;
                    end
            T3_XIND:begin
                    end
            T4_XIND:begin
                    ctl.ADLABL = 0; //hold ABL
                    end
            T5_XIND:begin
                    end
            T2_INDY:begin
                    end
            T3_INDY:begin
                    end
            T4_INDY:begin
                    end
            T5_INDY:begin
                    ctl.ADLABL = 0; //hold ABL
                    end
        endcase
    end

    control_view u_control_view(
        .ctl        (ctl)
    );


endmodule

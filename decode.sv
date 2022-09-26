`include "control.sv"

module decode(
    input  logic i_clk, i_rst,
    input  logic [7:0]	din,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,
    // input  logic i_alu_carry,

    output logic [7:0] ir,
    output logic SYNC,
    output st_ctl ctl
    );

    //internal control logic    
    //state machine
    logic [6:0] t, tlast;
    logic tskip;

    always @(posedge i_clk ) begin
        if (i_rst) begin
            t <= 7'b0000001;
            ir <= 8'b0;
            SYNC <= 1;
        end else begin
            ir <= ir;
            SYNC = 0;
            if (t == 7'b0000001) ir <= din;
                    
            if (t==tlast) begin
                t <= 7'b0000001;                
                SYNC <= 1;
            end else if (tskip) begin
                t <= t << 2;
            end else begin
                t <= t << 1;
            end 
        end
    end

    logic [7:0] current_op;
    assign current_op = SYNC ? din : ir;

    // https://www.masswerk.at/6502/6502_instruction_set.html#layout
    logic [2:0] op_a;
    logic [2:0] op_b;
    logic [1:0] op_c;
    assign {op_a, op_b, op_c} = current_op;


    // decode addr modes
    logic mode_abs, mode_absX, mode_imm, mode_impl;
    logic mode_Xind, mode_indY, mode_branch, mode_zpg, mode_zpgX;
    logic addr_idx; // 1 for X, 0 for y
    logic mode_jump;

    always @(*) begin
        mode_abs = 0;
        mode_absX = 0;
        mode_imm = 0;
        mode_impl = 0;
        mode_Xind = 0;
        mode_indY = 0;
        mode_zpg = 0;
        mode_zpgX = 0;
        addr_idx = 1;

        mode_jump = 0;
        mode_branch = 0;

        tskip = 0;
        tlast = 8'b100;

        case(op_b)
            0:  begin
                    if (op_c[0] == 0) begin          // c==0 or 2
                        if (op_a[2] == 0) begin      // a<4                        
                            mode_jump = 1;
                            mode_abs = op_a == 3'h1;
                            mode_impl = !mode_abs;
                        end
                        else
                            mode_imm = 1;
                    end else begin
                        mode_Xind = 1;              // c==1 or 3
                    end
                end
            1:  mode_zpg = 1;
            2:  begin
                    if (op_c[0] == 0)
                        mode_impl = 1; //includes mode A 
                    else
                        mode_imm = 1;
                end
            3:  begin
                    mode_abs = 1;
                    // todo: jump ind 
                    mode_jump = op_a == 3'b01X;
                end
            4:  begin
                    if (op_c[0] == 0)
                        mode_branch = 1; 
                    else begin
                        mode_indY = 1;
                        addr_idx = 0;
                    end
                end
            5:  begin
                    mode_zpgX = 1;
                    addr_idx = !(op_c[1] && (op_a[3:2] == 2'b10));
                end
            6:  begin
                    if (op_c[1] == 0)
                        mode_impl = 1;
                    else begin
                        mode_absX = 1;
                        addr_idx = 0;
                    end
                end
            7:  begin
                    mode_absX = 1;
                    addr_idx = !(op_c[1] && (op_a[3:2] == 2'b10));
                end
            // default:
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
                 ( current_op == 8'b11X_0X1_00 || current_op == 8'b11X_010_00 );

        op_sub  = (op_c[0] && (op_a == 7)) || op_cmp;

        op_asl = op_c[1] && (op_a == 0);
        op_rol = op_c[1] && (op_a == 1);
        op_lsr = op_c[1] && (op_a == 2);
        op_ror = op_c[1] && (op_a == 3);
        op_dec = op_c[1] && (op_a == 6);
        op_inc = (op_c[1] && (op_a == 7) && !(op_b==2) ) ||
                 ((op_c == 0) && (op_a[2:1] == 2'b11) && (op_b==2) );


        ctl.SUMS = op_add || op_sub;
        ctl.ANDS = 0;
        ctl.ORS = 0;
        ctl.EORS = 0;
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

        if (op_add || op_sub) begin
            ctl.ACRC = 1;
            ctl.DBZZ = 1;
        end
        //???

        ctl.IADDC = 0;

    end


    // db bus source
    // always @(*) begin
    initial begin
        ctl.DLDB = 1;
        ctl.PCLDB = 0;
        ctl.SBDB = 0;
        ctl.ACDB = 0;
        ctl.PDB = 0;
    end

    logic ADDADH;
    // sb bus source
    always @(*) begin
        ctl.XSB = !(t[0] || t[1]) && addr_idx;
        ctl.YSB = !(t[0] || t[1]) && !addr_idx;
        ctl.DBSB = 0;
        ctl.ADHSB = 0;

        ADDADH = (t[4] && mode_absX) ||
                        (t[5] && mode_indY);
        ctl.ADDSB0_6 = ADDADH;
        ctl.ADDSB7 = ADDADH;
        ctl.SSB = 0;
        ctl.ACSB = 0;
    end

    // adl bus source
    always @(*) begin
        ctl.PCLADL = t[0] || t[1] || (t[2] && (mode_abs || mode_absX));

        ctl.DLADL = t[2] && (mode_zpg || mode_indY);
        ctl.ADDADL = (t[3] && (mode_zpgX || mode_abs || mode_absX || mode_Xind || mode_indY)) ||
                 (t[4] && mode_indY) ||
                 (t[5] && mode_Xind);
        ctl.SADL = 0;
        ctl.ZADL0 = 0;
        ctl.ZADL1 = 0;
        ctl.ZADL2 = 0;
    end

    // adh bus source
    always @(*) begin
        ctl.PCHADH = t[0] || t[1] || (t[2] && (mode_abs || mode_absX));
        ctl.ZADH1_7 = t[2] && (mode_zpg || mode_indY) || 
                t[3] && (mode_zpgX || mode_Xind || mode_indY);
        ctl.ZADH0 = ctl.ZADH1_7;
        ctl.DLADH = (t[3] && (mode_abs || mode_absX)) ||
                (t[4] && (mode_indY || mode_Xind)) ||
                (t[5] && mode_Xind);

        ctl.SBADH = ADDADH;
    end

    // pc  source 
    always @(*) begin
        ctl.IPC = t[0] || t[1] || 
                  (t[2] && (mode_abs || mode_absX));
        ctl.PCLPCL = 0;
        ctl.PCHPCH = 0;

        ctl.ADLPCL = 0;
        ctl.ADHPCH = 0;
        ctl.PCLDB = 0;
        ctl.PCHDB = 0;
    end

    // ab source 
    always @(*) begin
        ctl.ADLABL = !(t[2] && (mode_zpgX || mode_Xind)) &&
                 !(t[4] && (mode_abs || mode_absX || mode_Xind)) &&
                 !(t[5] && mode_indY);

        ctl.ADHABH = !(t[2] && (mode_zpgX || mode_Xind));        
    end

    // alu source 
    always @(*) begin
        //bi sources
        ctl.ADLADD = t[3] && (mode_Xind);
        ctl.INVDBADD = 0;
        ctl.DBADD = !ctl.ADLADD && !ctl.INVDBADD;
        
        //ai sources
        ctl.SBADD = (t[2] && (mode_zpgX || mode_absX || mode_Xind)) ||
                (t[3] && mode_indY);
        ctl.ZADD = !ctl.SBADD;
    end


    // register load
    // always @(*) begin
    initial begin
        ctl.SBX = 0;
        ctl.SBY = 0;
        ctl.SBS = 0;
        ctl.SBAC = 0;
        ctl.SS = 0;
    end


    initial begin
        ctl.RW = 1;
        ctl.DAA = 0;
        ctl.DSA = 0;
    end

    control_view u_control_view(
        .ctl        (ctl)
    );


endmodule

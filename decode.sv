module decode(
    input  logic i_clk, i_rst,
    input  logic [7:0]	din,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,
    // input  logic i_alu_carry,

    output logic DLDB,       //input data latch to data bus
    output logic PCLDB,      //PCL to DB
    output logic PCHDB,      //PCH to DB
    output logic SBDB,       //SB to DB  (bidirection pass in schematic)
    output logic PDB,        //P to DB
    output logic ACDB,       //AC to DB
    output logic DLADL,      //input data latch to low address 
    output logic PCLADL,     //PCL to ADL
    output logic ZADL0,      //zero ADL0
    output logic ZADL1,      //zero ADL1
    output logic ZADL2,      //zero ADL2
    output logic SADL,       //stack to ADL
    output logic ADDADL,     //adder hold register to ADL
    output logic DLADH,      //input data latch to high address
    output logic ZADH0,      //zero ADH0
    output logic ZADH1_7,    //zero ADH1-7
    output logic PCHADH,     //PCH to ADH
    output logic SBADH,      //SB to ADH (bidirection pass in schematic)
    output logic SSB,        //load stack to SB
    output logic ADDSB0_6,   //ADD to SB
    output logic ADDSB7,     //ADD to SB
    output logic DBSB,       //DB to SB  (bidirection pass in schematic)
    output logic ADHSB,      //ADH to SB (bidirection pass in schematic)
    output logic XSB,        //X to SB
    output logic YSB,        //Y to SB
    output logic ACSB,       //accumulator to SB
    output logic SS,         //hold stack
    output logic SBS,        //load SB (register bus) to stack
    output logic ADHABH,     //ADH to ABH register (load on ph1)
    output logic ADLABL,     //ADL to ABL register (load on ph1)
    output logic PCLPCL,     //load PCL into PCL select register
    output logic ADLPCH,     //load ADL into PCL select register
    output logic PCHPCH,     //load PCH into PCH select register
    output logic ADHPCH,     //load ADH into PCH select register
    output logic IPC,        //increment PC
    output logic SBAC,       //SB to AC (via decimal adjust logic)
    output logic SBX,        //SB to X
    output logic SBY,        //SB to Y
    output logic SBADD,      //SB bus
    // output logic ZADD,       //add zero (not needed??, default ai=0 if SBADD=0) 
    output logic INVDBADD,   //load inv DB to ALU B
    output logic DBADD,      //load DB to ALU B
    output logic ADLADD,     //load ADL to ALU B
    output logic IADDC,      //carry in 
    output logic DAA,         //decimal enable 
    output logic SUMS,        //SUM
    output logic ANDS,        //AND
    output logic EORS,        //XOR
    output logic ORS,         //OR
    output logic SRS,         //SRR
    // output logic DSA,         //decimal adjust (NA)
    output logic DBZC,       //DB0 -> carry bit
    output logic IR5C,       //IR5 -> carry bit
    output logic ACRC,       //ACR -> carry bit
    output logic DB1Z,       //DB1 -> Z bit
    output logic DBZZ,       //DB=0 -> Z bit
    output logic DB2I,       //DB2 -> I bit
    output logic IR5I,       //IR5 -> I bit
    output logic DB3D,       //DB3 -> D bit      
    output logic IR5D,       //IR5 -> D bit
    output logic DB6V,       //DB6 -> V bit
    output logic AVRV,       //AVR -> V bit
    output logic IV,         //I bit -> V bit  (???)
    output logic DB7N,       //DB7 -> N bit
    output logic RWbar,
    output logic sync,
    );


    //internal control logic
    logic last, tskip;
    
    //state machine
    logic [6:0] t;
    always @(posedge i_clk ) begin
        if i_rst begin
            t <= '1;
            ir <= '0;
            sync <= 1;
        end else begin
            ir = ir;
            sync = 0;
            if t == '1 begin
                ir = din;    
            end else if last begin
                t <= '1;                
                sync = 1;
            end else if tskip begin
                t <= t << 2;
            end else begin
                t <= t << 1;
            end 
        end
    end

    logic [7:0] current_op;
    assign current_op = sync ? din : ir;

    // https://www.masswerk.at/6502/6502_instruction_set.html#layout
    logic [2:0] op_a;
    logic [2:0] op_b;
    logic [1:0] op_c;
    assign {op_a, op_b, op_c} = current_op;


    // decode addr modes
    logic mode_abs, mode_absX, mode_imm, mode_impl;
    logic mode_Xind, mode_indY, mode_rel, mode_zpg, mode_zpgX;
    logic addr_idx; // 1 for X, 0 for y
    logic mode_special;

    always_comb begin
        mode_abs = 0;
        mode_absX = 0;
        mode_imm = 0;
        mode_impl = 0;
        mode_Xind = 0;
        mode_indY = 0;
        mode_rel = 0;
        mode_zpg = 0
        mode_zpgX = 0;
        mode_special = 0;
        addr_idx = 1; //default to x

        case(op_b)
            0:  begin
                    if op_c[0] == 0 begin
                        mode_special = op_a[2] == 0;
                        mode_imm = op_a[2] == 1;
                    end else mode_Xind = 1;
                    end
                end
            1:  mode_zpg = 1;
            2:  begin
                    if op_c[1] == 0
                        mode_impl = 1; //includes mode A 
                    else
                        mode_imm = 1;
                    end
                end
            3: 
            3:  mode_abs = 1;
            4:  begin
                    if op_c[1] == 0
                        mode_rel = 1; 
                    else
                        mode_indY = 1;
                        addr_idx = 0;
                    end
                end
            5:  begin
                    mode_zpgX = 1;
                    addr_idx = !(op_c[1] && (op_a[3:2] == 2'b10));
                end
            6:  begin
                    if op_c[1] == 0
                        mode_impl = 1;
                    else
                        mode_absX = 1;
                        addr_idx = 0;
                    end
                end
            7:  begin
                    mode_absX = 1;
                    addr_idx = !(op_c[1] && (op_a[3:2] == 2'b10));
                end
            default:
        endcase
    end

    logic op_or, op_and, op_xor, op_add, op_sub, op_cmp;
    logic op_asl, op_rol, op_lsr, op_ror, op_dec, op_inc;
    //decode alu operation
    always_comb begin
        op_or = op_c[0] && op_a == 0;
        op_and = op_c[0] && op_a == 1;
        op_xor = op_c[0] && op_a == 2;
        op_add = op_c[0] && op_a == 3;
        op_cmp = ( op_c[0] && op_a == 6 ) ||
                 ( current_op == 8'b11X_0X1_00 || current_op == 8'b11X_010_00 )

        op_sub  = (op_c[0] && (op_a == 7)) || op_cmp;

        op_asl = op_c[1] && (op_a == 0);
        op_rol = op_c[1] && (op_a == 1);
        op_lsr = op_c[1] && (op_a == 2);
        op_ror = op_c[1] && (op_a == 3);
        op_dec = op_c[1] && (op_a == 6);
        op_inc = (op_c[1] && (op_a == 7) && !(op_b==2) ) ||
                 ((op_c == 0) && (op_a[2:1] == 2'b11) && (op_b==2) )


        SUMS = op_add || op_sub;
        // .todo

        DB0C = 0;
        DB1Z = 0;
        DBZZ = 0;
        DB2I = 0;
        DB3D = 0;
        DB6V = 0;
        DB7N = 0;

        IR5C = 0;
        IR5V = 0;
        IR5D = 0;

        ACRC = 0;
        AVRV = 0;
        IV = 0;

        if (op_add || op_sub) begin
            ACRC = 1;
            DBZZ = 1;
        end
        //???



    end


    // db bus source
    always_comb begin
        DLDB = 1;
        PCLDB = 0;
        SBDB = 0;
        ACDB = 0;
        PDB = 0;
    end

    // sb bus source
    always_comb begin
        XSB = !(t[0] || t[1]) && addr_idx
        YSB = !(t[0] || t[1]) && !addr_idx
        DBSB = 0;
        ADHSB = 0;
        SSB = 0;
        ADDSB = 0;
        ACSB = 0;
    end

    // adl bus source
    always_comb begin
        PCLADL = t[0] || t[1] || (t[2] && (mode_abs || mode_absX));

        DLADL = t[2] && (mode_zpg || mode_indY);
        ADDADL = (t[3] && (mode_zpgX || mode_abs || mode_absX || mode_Xind || mode_indY)) ||
                 (t[4] && mode_indY) ||
                 (t[5] && mode_Xind);
        SADL = 0;
        ZADL0 = 0;
        ZADL1 = 0;
        ZADL2 = 0;
    end

    // adh bus source
    always_comb begin
        PCHADH = t[0] || t[1] || (t[2] && (mode_abs || mode_absX));
        ZADH1_7 = t[2] && (mode_zpg || mode_indY) || 
                t[3] && (mode_zpgX || mode_Xind || mode_indY);
        ZADH0 = ZADH1_7;
        DLADH = (t[3] && (mode_abs || mode_absX)) ||
                (t[4] && mode_indY) ||
                (t[5] && mode_Xind);

        ADDADH = (t[4] && (mode_abs || mode_absX || mode_Xind)) ||
                 (t[5] && mode_indY);

        SBADH = 0;
    end

    // pc  source 
    always_comb begin
        IPC = t[0] || t[1] || 
                  (t[2] && (mode_abs || mode_absX));
        ADLPCL = 0;
        ADHPCH = 0;
    end

    // ab source 
    always_comb begin
        ADLABL = !(t[2] && (mode_zpgX || mode_Xind) &&
                 !(t[4] && (mode_abs || mode_absX || mode_Xind) &&
                 !(t[5] && mode_indY);

        ADHABH = !(t[2] && (mode_zpgX || mode_Xind);        
    end

    // alu source 
    always_comb begin
        //bi sources
        ADLADD = t[3] && (mode_Xind);
        INVDBADD = 0;
        DBADD = !ADLADD && !INVDBADD;
        
        //ai sources
        SBADD = (t[2] && (mode_zpgX || mode_absX || mode_Xind)) ||
                (t[3] && mode_indY);
        ZADD = !SBADD;
    end

endmodule

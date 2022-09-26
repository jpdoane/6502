module 6502(
		input	wire	        i_clk,
		input	wire    [7:0]	i_data,
		output	wire    [7:0]	o_data,
		output	wire    [15:0]	o_addr
    );

    // registers
    logic [7:0] dl, dor, ir;
    logic [7:0] pcl, pch;
    logic [7:0] add, p;
    logic [7:0] a, s, x, y;

    assign o_data = dor;
    assign o_addr = addr;
    assign dl = i_data;

    // buses
    logic [7:0] db, sb, adl, adh;

    //control
    logic DLDB;       //input data latch to data bus
    logic PCLDB;      //PCL to DB
    logic PCHDB;      //PCH to DB
    logic SBDB;       //SB to DB  (bidirection pass in schematic)
    logic PDB;        //P to DB
    logic ACDB;       //AC to DB
    logic DLADL;      //input data latch to low address 
    logic PCLADL;     //PCL to ADL
    logic ZADL0;      //zero ADL0
    logic ZADL1;      //zero ADL1
    logic ZADL2;      //zero ADL2
    logic SADL;       //stack to ADL
    logic ADDADL;     //adder hold register to ADL
    logic DLADH;      //input data latch to high address
    logic ZADH0;      //zero ADH0
    logic ZADH1_7;    //zero ADH1-7
    logic PCHADH;     //PCH to ADH
    logic SBADH;      //SB to ADH (bidirection pass in schematic)
    logic SSB;        //load stack to SB
    logic ADDSB0_6;   //ADD to SB
    logic ADDSB7;     //ADD to SB
    logic DBSB;       //DB to SB  (bidirection pass in schematic)
    logic ADHSB;      //ADH to SB (bidirection pass in schematic)
    logic XSB;        //X to SB
    logic YSB;        //Y to SB
    logic ACSB;       //accumulator to SB
    logic SS;         //hold stack
    logic SBS;        //load SB (register bus) to stack
    logic ADHABH;     //ADH to ABH register (load on ph1)
    logic ADLABL;     //ADL to ABL register (load on ph1)
    logic PCLPCL;     //load PCL into PCL select register
    logic ADLPCH;     //load ADL into PCL select register
    logic PCHPCH;     //load PCH into PCH select register
    logic ADHPCH;     //load ADH into PCH select register
    logic IPC;        //increment PC
    logic SBAC;       //SB to AC (via decimal adjust logic)
    logic SBX;        //SB to X
    logic SBY;        //SB to Y
    logic SBADD;      //SB bus
    // logic ZADD;       //add zero (not needed??, default ai=0 if SBADD=0) 
    logic INVDBADD;   //load inv DB to ALU B
    logic DBADD;      //load DB to ALU B
    logic ADLADD;     //load ADL to ALU B
    logic IADDC;      //carry in 
    // logic DAA;         //decimal enable 
    logic SUMS;        //SUM
    logic ANDS;        //AND
    logic EORS;        //XOR
    logic ORS;         //OR
    logic SRS;         //SRR
    logic AVR;         //overflow
    logic ACR;         //carry
    // logic HC;          //halfcarry
    // logic DSA;         //decimal adjust (NA)
    logic DBZC;       //DB0 -> carry bit
    logic IR5C;       //IR5 -> carry bit
    logic ACRC;       //ACR -> carry bit
    logic DB1Z;       //DB1 -> Z bit
    logic DBZZ;       //DB=0 -> Z bit
    logic DB2I;       //DB2 -> I bit
    logic IR5I;       //IR5 -> I bit
    logic DB3D;       //DB3 -> D bit      
    logic IR5D;       //IR5 -> D bit
    logic DB6V;       //DB6 -> V bit
    logic AVRV;       //AVR -> V bit
    logic IV;         //I bit -> V bit  (???)
    logic DB7N;       //DB7 -> N bit
    logic READY;
    logic SV;
    logic NMI;
    logic IRQ;
    logic RES;
    logic RWbar;
    logic SYNC;

    // output address 
    always @(*) begin
        addr[15:8] = ADHABH ? adh : 8'b0;
        addr[7:0] = ADLABL ? adl : 8'b0;        
    end

    // register updates
    always @(posedge i_clk ) begin
        a <= SBAC ? sb : a;
        s <= SBS ? sb : s;
        x <= SBX ? sb : x;
        y <= SBY ? sb : y;
        dor <= WE ? db : 8'b0;
    end

    //bus control
    // buses may be tied together bi-directionally
    // to do this on FPGA w/o tristate logic, form intermediate muxes for each bus
    // which can be combined as needed without creating a latch
    logic [7:0] db_mux, sb_mux, adl_mux, adh_mux;
    always @(*) begin
        //multiple drivers are and-ed to reflect original 6502 behavior and support "illegal" opcodes

        adl_mux = {5'b1, !ZADL2,!ZADL1,!ZADL0};
        if DLADL adl_mux &= pcl;
        if ADDADL adl_mux &= add;
        //if SBS & SADL, then sb is immediately fed forward
        //to adl instead of the previously registered stack
        if SADL adl_mux &= (SBS ? sb : s)

        adh_mux = !{7{ZADH1_7},ZADH0}
        if DLADH adh_mux &= dl;
        if PCHADH adh_mux &= pch;

        db_mux = 8'hff;
        if DLDB db_mux &= dl;
        if PCLDB db_mux &= pcl;
        if PCHDB db_mux &= pch;
        if PDB db_mux &= p;
        if ACDB db_mux &= a;

        sb_mux = {ADDSB7 ? add[7] : 1'b1, ADDSB0_6 ? add[6:0] : 7'b1}
        if SBADH sb_mux &= adh;
        if SSB sb_mux &= s;
        if XSB sb_mux &= x;
        if YSB sb_mux &= y;
        if ACSB sb_mux &= a;

        // tie busses together (AND)
        sb = sb_mux
        if SBDB sb &= db_mux
        if SBADH sb &= adh_mux

        db = db_mux
        if SBDB db &= sb_mux
        if (SBDB & SBADH) db &= adh_mux

        adh = adh_mux
        if SBADH adh &= sb_mux
        if (SBDB & SBADH) adh &= db_mux

        adl = adl_mux;
    end

    //alu
    logic [7:0] alu_ai, alu_bi, alu_out;
    logic [7:0] alu_sum, alu_and, alu_xor, alu_or, alu_sr, alu_out;
    logic alu_sum_carry, alu_sr_carry;
    always @(*) begin
        alu_ai =    SBADD ? sb : 8'b0;
        alu_bi =    ADLADD ? adl :
                    DBADD ? db :
                    INVDBADD ? !db :
                    8'b0;
        
        {alu_sum_carry, alu_sum} = alu_ai + alu_bi + IADDC;
        alu_and = alu_ai & alu_bi;
        alu_or = alu_ai | alu_bi;
        alu_xor = alu_ai ^ alu_bi;
        { alu_sr, alu_sr_carry } = { IADDC, alu_ai };

        alu_out = 8'b0;
        if(SUMS) alu_out |= alu_sum;
        if(ANDS) alu_out |= alu_and;
        if(ORS) alu_out |= alu_or;
        if(EORS) alu_out |= alu_xor;
        if(SRS) alu_out |= alu_sr;
        
        ACR = 1'b0;
        if(SUMS) ACR |= alu_sum_carry;
        if(SRS) ACR |= alu_sr_carry;

        //todo: overflow, hc...
        AVR = 1'b0;

    end

    // register alu result
    always @(posedge i_clk ) begin        
        add <= alu_out;
    end

    // status reg
    logic _c, _z, _i, _d, _v, _n
    always @(*) begin
        _c =    DBZC ? db[0] :
                IR5C ? ir[5] :
                ACRC ? ACR :
                p[0];

        _z =    DB1Z ? db[1] :
                DBZZ ? (dbz == 0) :
                p[1];

        _i =    DB2I ? db[2] :
                IR5I ? ir[5] :
                p[2];
        
        _d =    DB3D ? db[3] :
                IR5D ? ir[5] :
                p[3];

        _v =    DB6V ? db[6] :
                AVRV ? AVR :
                p[6];

        _n =    DB7N ? db[7] :
                p[7];
    end
    always @(posedge i_clk ) begin        
        p <= {_n, _v, 1'b0, 1'b0, _d, _i, _z, _c};
    end

module core_simple(
    input  logic i_clk, i_rst,
    input  logic [7:0] din,

    output logic [15:0] addr,
    output logic [7:0] dout,
    output logic RW
    );

    assign RW=1;

    logic DLDB = 0;
    logic PCLDB = 0;
    logic PCHDB = 0;
    logic SBDB = 0;
    logic PDB = 0;
    logic ACDB = 0;
    logic DLADL = 0;
    logic PCLADL = 0;
    logic ZADL0 = 0;
    logic ZADL1 = 0;
    logic ZADL2 = 0;
    logic SADL = 0;
    logic ADDADL = 0;
    logic DLADH = 0;
    logic ZADH0 = 0;
    logic ZADH1_7 = 0;
    logic PCHADH = 0;
    logic SBADH = 0;
    logic SSB = 0;
    logic ADDSB0_6 = 0;
    logic ADDSB7 = 0;
    logic DBSB = 0;
    logic ADHSB = 0;
    logic XSB = 0;
    logic YSB = 0;
    logic ACSB = 0;
    logic SS = 0;
    logic SBS = 0;
    logic ADHABH = 0;
    logic ADLABL = 0;
    logic PCLPCL = 0;
    logic ADLPCH = 0;
    logic PCHPCH = 0;
    logic ADHPCH = 0;
    logic IPC = 0;
    logic SBAC = 0;
    logic SBX = 0;
    logic SBY = 0;
    logic SBADD = 0;
    logic INVDBADD = 0;
    logic DBADD = 0;
    logic ADLADD = 0;
    logic IADDC = 0;
    logic DAA = 0;
    logic SUMS = 0;
    logic ANDS = 0;
    logic EORS = 0;
    logic ORS = 0;
    logic SRS = 0;
    logic DBZC = 0;
    logic IR5C = 0;
    logic ACRC = 0;
    logic DB1Z = 0;
    logic DBZZ = 0;
    logic DB2I = 0;
    logic IR5I = 0;
    logic DB3D = 0;
    logic IR5D = 0;
    logic DB6V = 0;
    logic AVRV = 0;
    logic IV = 0;
    logic DB7N = 0;


    // registers
    logic [7:0] dl, dor, ir;
    logic [7:0] add, p;
    logic [7:0] a, s, x, y;
    logic [7:0] ai, bi;

    logic [7:0] pcl, pch;
    assign pcl = pc[0:7];
    assign pch = pc[15:8];

    // buses
    logic [7:0] db, sb, adl, adh;

    assign o_data = dor;
    assign o_addr = addr;
    assign dl = i_data;


    // output address (pseudolatch)
    // setting ADLABL/ADHABH updates addr immediately
    // otherwise, value is obtained from registered copy (hold)
    logic [15:0] raddr;
    assign addr=16'b0;

    // always_comb begin
    //     addr[15:8] = ADHABH ? adh : raddr[15:8];
    //     addr[7:0] = ADLABL ? adl : raddr[7:0];        
    // end

    // register updates
    logic [7:0] rs;
    assign s = SBS ? sb : rs; // s updated immediatly (pseudo-latch)
    always @(posedge i_clk ) begin
        a <= SBAC ? sb : a;
        x <= SBX ? sb : x;
        y <= SBY ? sb : y;
        dor <= WE ? db : 8'b0;

        //registers for pseudo-latch
        raddr <= addr;
        rs <= s;
    end

    // pc
    logic [15:0] pcs;
    logic [7:0] pchs, pcls;
    assign pcls = ADLPCL ? adl : pcl;
    assign pchs = ADHPCH ? adh : pch;
    assign pcs = {pchs, pcls};
    always @(posedge i_clk ) begin
        pc <= IPC ? pcs+1 : pcs;
    end


    //bus control
    // buses may be tied together bi-directionally
    // to do this on FPGA w/o tristate logic, form intermediate muxes for each bus
    // which can be combined as needed without creating a latch
    logic [7:0] db_mux, sb_mux, adl_mux, adh_mux;
    always_comb begin
        //multiple drivers are and-ed to reflect original 6502 behavior and support "illegal" opcodes

        adl_mux = {5'b1, !ZADL2,!ZADL1,!ZADL0};
        if (DLADL) adl_mux &= pcl;
        if (ADDADL) adl_mux &= add;
        //if SBS & SADL, then sb is immediately fed forward
        //to adl instead of the previously registered stack
        if SADL adl_mux &= (SBS ? sb : s)

        adh_mux = !{7{ZADH1_7},ZADH0}
        if DLADH adh_mux &= dl;
        if PCHADH adh_mux &= pch;

        db_mux = 8'hff;
        if (DLDB) db_mux &= dl;
        if (PCLDB) db_mux &= pcl;
        if (PCHDB) db_mux &= pch;
        if (PDB) db_mux &= p;
        if (ACDB) db_mux &= a;

        sb_mux = {ADDSB7 ? add[7] : 1'b1, ADDSB0_6 ? add[6:0] : 7'b1}
        if (SBADH) sb_mux &= adh;
        if (SSB) sb_mux &= s;
        if (XSB) sb_mux &= x;
        if (YSB) sb_mux &= y;
        if (ACSB) sb_mux &= a;

        // tie busses together (AND)
        sb = sb_mux
        if (SBDB) sb &= db_mux
        if (SBADH) sb &= adh_mux

        db = db_mux
        if (SBDB) db &= sb_mux
        if (SBDB & SBADH) db &= adh_mux

        adh = adh_mux
        if (SBADH) adh &= sb_mux
        if (SBDB & SBADH) adh &= db_mux

        adl = adl_mux;
    end

    //alu inputs
    always_comb begin
        ai =  8'b1;
        bi =  8'b1;
        add = 8'b1;

        // a input mux
        if(SBADD) ai &= sb;
        if(ZADD) ai &= 8'b0;

        // b input mux
        if(DBADD) bi &= db;
        if(INVDBADD) bi &= !db;
        if(ADLADD) bi &= adl;
    end

    logic [7:0] add;
    logic ACR, AVR;
    always @(posedge i_clk ) begin
        add <= add;
        ACR <= 0;
        AVR <= 0; //todo
        if (SUMS) {ACR, add} <= ai + bi + IADDC;
        if (ANDS) add <= ai & bi;
        if (ORS) add <= ai | bi;
        if (EORS) add <= ai ^ bi;
        if (SRS) {add, ACR} <= {IADDC, ai}
    end


    // processor status reg
    always @(posedge i_clk ) begin
        p <= p;        
        if(DB0C) p[0] <= db[0]; 
        if(IR5C) p[0] <= ir[5];
        if(ACRC) p[0] <= ACR;

        if(DB1Z) p[1] <= db[1];
        if(DBZZ) p[1] <= (db == '0);

        if(DB2I) p[2] <= db[2];
        if(IR5I) p[2] <= ir[5];

        if(DB3D) p[3] <= db[3];
        if(IR5D) p[3] <= ir[5];

        if(DB6V) p[6] <= db[6];
        if(AVRV) p[6] <= AVR;
        if(IV)   p[6] <= 0; // ????

        if(DB7N) p[7] <= db[7];
    end

endmodule

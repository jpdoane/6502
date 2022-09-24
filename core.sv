module core(
    input  logic i_clk, i_rst,
    input  logic [7:0] din,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,
    input  logic i_alu_carry,

    output logic [15:0] addr,
    output logic [7:0] dout,
    output logic RW
    );

    // registers
    logic [7:0] dl, dor, ir;
    logic [7:0] add, p;
    logic [7:0] a, s, x, y;

    logic [7:0] pcl, pch;
    assign pcl = pc[0:7];
    assign pch = pc[15:8];

    // buses
    logic [7:0] db, sb, adl, adh;

    // alu in/out
    logic [7:0] ai, bi, add;

    assign o_data = dor;
    assign o_addr = addr;
    assign dl = i_data;



    decode u_decode(
    	.i_clk    (i_clk    ),
        .i_rst    (i_rst    ),
        .din      (din      ),
        .READY    (READY    ),
        .SV       (SV       ),
        .NMI      (NMI      ),
        .IRQ      (IRQ      ),
        .DLDB     (DLDB     ),
        .PCLDB    (PCLDB    ),
        .PCHDB    (PCHDB    ),
        .SBDB     (SBDB     ),
        .PDB      (PDB      ),
        .ACDB     (ACDB     ),
        .DLADL    (DLADL    ),
        .PCLADL   (PCLADL   ),
        .ZADL0    (ZADL0    ),
        .ZADL1    (ZADL1    ),
        .ZADL2    (ZADL2    ),
        .SADL     (SADL     ),
        .ADDADL   (ADDADL   ),
        .DLADH    (DLADH    ),
        .ZADH0    (ZADH0    ),
        .ZADH1_7  (ZADH1_7  ),
        .PCHADH   (PCHADH   ),
        .SBADH    (SBADH    ),
        .SSB      (SSB      ),
        .ADDSB0_6 (ADDSB0_6 ),
        .ADDSB7   (ADDSB7   ),
        .DBSB     (DBSB     ),
        .ADHSB    (ADHSB    ),
        .XSB      (XSB      ),
        .YSB      (YSB      ),
        .ACSB     (ACSB     ),
        .SS       (SS       ),
        .SBS      (SBS      ),
        .ADHABH   (ADHABH   ),
        .ADLABL   (ADLABL   ),
        .PCLPCL   (PCLPCL   ),
        .ADLPCH   (ADLPCH   ),
        .PCHPCH   (PCHPCH   ),
        .ADHPCH   (ADHPCH   ),
        .IPC      (IPC      ),
        .SBAC     (SBAC     ),
        .SBX      (SBX      ),
        .SBY      (SBY      ),
        .SBADD    (SBADD    ),
        .INVDBADD (INVDBADD ),
        .DBADD    (DBADD    ),
        .ADLADD   (ADLADD   ),
        .IADDC    (IADDC    ),
        .DAA      (DAA      ),
        .SUMS     (SUMS     ),
        .ANDS     (ANDS     ),
        .EORS     (EORS     ),
        .ORS      (ORS      ),
        .SRS      (SRS      ),
        .DBZC     (DBZC     ),
        .IR5C     (IR5C     ),
        .ACRC     (ACRC     ),
        .DB1Z     (DB1Z     ),
        .DBZZ     (DBZZ     ),
        .DB2I     (DB2I     ),
        .IR5I     (IR5I     ),
        .DB3D     (DB3D     ),
        .IR5D     (IR5D     ),
        .DB6V     (DB6V     ),
        .AVRV     (AVRV     ),
        .IV       (IV       ),
        .DB7N     (DB7N     ),
        .RWbar    (RWbar    ),
        .sync     (sync     )
    );




    // output address (pseudolatch)
    // setting ADLABL/ADHABH updates addr immediately
    // otherwise, value is obtained from registered copy (hold)
    logic [15:0] raddr;
    always_comb begin
        addr[15:8] = ADHABH ? adh : raddr[15:8];
        addr[7:0] = ADLABL ? adl : raddr[7:0];        
    end

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

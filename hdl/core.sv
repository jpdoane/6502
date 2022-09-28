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
    output logic [7:0] o_data,
    output logic RW
    );

    logic [7:0] dor, dl, ir;
    assign o_data = dor;
    assign dl = i_data;

    logic ACR, AVR;
    st_ctl ctl;

    control u_control(
    	.i_clk    (i_clk    ),
        .i_rst    (i_rst    ),
        .din      (dl      ),
        .READY    (READY    ),
        .SV       (SV       ),
        .NMI      (NMI      ),
        .IRQ      (IRQ      ),
        .alu_carry (ACR),
        .ir       (ir),
        .ctl      (ctl)
    );


    // registers
    logic [7:0] add, p;
    logic [7:0] a, s, x, y;
    logic [7:0] ai, bi;

    logic [15:0] pc;
    logic [7:0] pcl, pch;
    assign pcl = pc[7:0];
    assign pch = pc[15:8];

    // buses
    logic [7:0] db, sb, adl, adh;



    // output address (pseudolatch)
    // setting ADLABL/ADHABH updates addr immediately
    // otherwise, value is obtained from registered copy (hold)
    logic [15:0] raddr;
    always @(*) begin
    // always @(*) begin
        addr[15:8] = ctl.ADHABH ? adh : raddr[15:8];
        addr[7:0] = ctl.ADLABL ? adl : raddr[7:0];        
    end

    // register updates
    // update s immediatly
    logic [7:0] rs;
    assign s = ctl.SBS ? sb : rs; 
    always @(posedge i_clk ) begin
        if (i_rst) begin
            a <= 8'b0;
            x <= 8'b0;
            y <= 8'b0;
            rs <= 8'b0;
            dor <= 8'b0;
            raddr <= BOOT_ADDR;
        end else begin
            a <= ctl.SBAC ? sb : a;
            x <= ctl.SBX ? sb : x;
            y <= ctl.SBY ? sb : y;
            rs <= s;
            dor <= !ctl.RW ? db : 8'b0;
            raddr <= addr;
        end
    end

    // pc
    logic [15:0] pcs;
    logic [7:0] pchs, pcls;
    assign pcls = ctl.ADLPCL ? adl : pcl;
    assign pchs = ctl.ADHPCH ? adh : pch;
    assign pcs = {pchs, pcls};
    always @(posedge i_clk ) begin
        if (i_rst) begin
            pc <= 8'b0;
        end else begin
            pc <= ctl.IPC ? pcs+1 : pcs;
        end
    end


    //bus control
    // buses may be tied together bi-directionally
    // to do this on FPGA w/o tristate logic, form intermediate muxes for each bus
    // which can be combined as needed without creating a latch
    logic [7:0] db_mux, sb_mux, adl_mux, adh_mux;
    always @(*) begin
        //multiple drivers are and-ed to reflect original 6502 behavior and support "illegal" opcodes

        adl_mux = {5'b11111, ~ctl.ZADL2,~ctl.ZADL1,~ctl.ZADL0};
        if (ctl.DLADL) adl_mux &= dl;
        if (ctl.PCLADL) adl_mux &= pcl;
        if (ctl.ADDADL) adl_mux &= add;
        //if SBS & SADL, then sb is immediately fed forward
        //to adl instead of the previously registered stack
        if (ctl.SADL) adl_mux &= (ctl.SBS ? sb : s);

        adh_mux = ~{ {7{ctl.ZADH1_7}}, ctl.ZADH0};
        if (ctl.DLADH) adh_mux &= dl;
        if (ctl.PCHADH) adh_mux &= pch;

        db_mux = 8'hff;
        if (ctl.DLDB) db_mux &= dl;
        if (ctl.PCLDB) db_mux &= pcl;
        if (ctl.PCHDB) db_mux &= pch;
        if (ctl.PDB) db_mux &= p;
        if (ctl.ACDB) db_mux &= a;

        sb_mux = 8'hff;
        if (ctl.ADDSB7) sb_mux[7] &= add[7];
        if (ctl.ADDSB0_6) sb_mux[6:0] &= add[6:0];
        // if we are simultaneously writing and reading to/from reg,
        // then ignore the read, since data is already on the bus...
        if (ctl.SSB && !ctl.SBS) sb_mux &= s;
        if (ctl.XSB && !ctl.SBX) sb_mux &= x;
        if (ctl.YSB && !ctl.SBY) sb_mux &= y;
        if (ctl.ACSB && !ctl.SBAC) sb_mux &= a;

        // tie busses together (AND)
        sb = sb_mux;
        if (ctl.SBDB) sb &= db_mux;
        if (ctl.SBADH) sb &= adh_mux;

        db = db_mux;
        if (ctl.SBDB) db &= sb_mux;
        if (ctl.SBDB & ctl.SBADH) db &= adh_mux;

        adh = adh_mux;
        if (ctl.SBADH) adh &= sb_mux;
        if (ctl.SBDB & ctl.SBADH) adh &= db_mux;

        adl = adl_mux;
    end

    //alu inputs
    always @(*) begin
    // always @(*) begin
        ai =  8'hff;
        bi =  8'hff;

        // a input mux
        if(ctl.SBADD) ai &= sb;
        if(ctl.ZADD) ai &= 8'b0;

        // b input mux
        if(ctl.DBADD) bi &= db;
        if(ctl.INVDBADD) bi &= ~db;
        if(ctl.ADLADD) bi &= adl;
    end

    always @(posedge i_clk ) begin
        if (i_rst) begin
            add <= 8'b0;
            ACR <= 1'b0;
            AVR <= 1'b0;
        end else begin
            add <= add;
            ACR <= 0;
            AVR <= 0; //todo
            if (ctl.SUMS) {ACR, add} <= ai + bi + ctl.IADDC;
            if (ctl.ANDS) add <= ai & bi;
            if (ctl.ORS) add <= ai | bi;
            if (ctl.EORS) add <= ai ^ bi;
            if (ctl.SRS) {add, ACR} <= {ctl.IADDC, ai};
        end
    end


    // processor status reg
    always @(posedge i_clk ) begin
        if (i_rst) begin
            p <= 8'b0;
        end else begin
            p <= p;        
            if(ctl.DB0C) p[0] <= db[0]; 
            if(ctl.IR5C) p[0] <= ir[5];
            if(ctl.ACRC) p[0] <= ACR;

            if(ctl.DB1Z) p[1] <= db[1];
            if(ctl.DBZZ) p[1] <= (db == 8'b0);

            if(ctl.DB2I) p[2] <= db[2];
            if(ctl.IR5I) p[2] <= ir[5];

            if(ctl.DB3D) p[3] <= db[3];
            if(ctl.IR5D) p[3] <= ir[5];

            if(ctl.DB6V) p[6] <= db[6];
            if(ctl.AVRV) p[6] <= AVR;
            if(ctl.IV)   p[6] <= 0; // ????

            if(ctl.DB7N) p[7] <= db[7];
        end
    end

endmodule

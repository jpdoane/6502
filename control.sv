typedef struct packed {
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
    logic ADLPCL;     //load ADL into PCL select register
    logic PCHPCH;     //load PCH into PCH select register
    logic ADHPCH;     //load ADH into PCH select register
    logic IPC;        //increment PC
    logic SBAC;       //SB to AC (via decimal adjust logic)
    logic SBX;        //SB to X
    logic SBY;        //SB to Y
    logic SBADD;      //SB bus
    logic ZADD;       //add zero (not needed??, default ai=0 if SBADD=0) 
    logic INVDBADD;   //load inv DB to ALU B
    logic DBADD;      //load DB to ALU B
    logic ADLADD;     //load ADL to ALU B
    logic IADDC;      //carry in 
    logic DAA;         //decimal enable 
    logic SUMS;        //SUM
    logic ANDS;        //AND
    logic EORS;        //XOR
    logic ORS;         //OR
    logic SRS;         //SRR
    logic DSA;         //decimal adjust (NA)
    logic DB0C;       //DB0 -> carry bit
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
    logic IR5V;       //AVR -> V bit
    logic IV;         //I bit -> V bit  (???)
    logic DB7N;       //DB7 -> N bit
    logic RW;
    logic sync;
} st_ctl;

//dummy module to view individual control signals in GTKwave
module control_view(
    input st_ctl ctl
    );

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
    logic ADLPCL;     //load ADL into PCL select register
    logic PCHPCH;     //load PCH into PCH select register
    logic ADHPCH;     //load ADH into PCH select register
    logic IPC;        //increment PC
    logic SBAC;       //SB to AC (via decimal adjust logic)
    logic SBX;        //SB to X
    logic SBY;        //SB to Y
    logic SBADD;      //SB bus
    logic ZADD;       //add zero (not needed??, default ai=0 if SBADD=0) 
    logic INVDBADD;   //load inv DB to ALU B
    logic DBADD;      //load DB to ALU B
    logic ADLADD;     //load ADL to ALU B
    logic IADDC;      //carry in 
    logic DAA;         //decimal enable 
    logic SUMS;        //SUM
    logic ANDS;        //AND
    logic EORS;        //XOR
    logic ORS;         //OR
    logic SRS;         //SRR
    logic DSA;         //decimal adjust (NA)
    logic DB0C;       //DB0 -> carry bit
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
    logic IR5V;       //AVR -> V bit
    logic IV;         //I bit -> V bit  (???)
    logic DB7N;       //DB7 -> N bit
    logic RW;
    logic sync;

    assign DLDB = ctl.DLDB;       //input data latch to data bus
    assign PCLDB = ctl.PCLDB;      //PCL to DB
    assign PCHDB = ctl.PCHDB;      //PCH to DB
    assign SBDB = ctl.SBDB;       //SB to DB  (bidirection pass in schematic)
    assign PDB = ctl.PDB;        //P to DB
    assign ACDB = ctl.ACDB;       //AC to DB
    assign DLADL = ctl.DLADL;      //input data latch to low address 
    assign PCLADL = ctl.PCLADL;     //PCL to ADL
    assign ZADL0 = ctl.ZADL0;      //zero ADL0
    assign ZADL1 = ctl.ZADL1;      //zero ADL1
    assign ZADL2 = ctl.ZADL2;      //zero ADL2
    assign SADL = ctl.SADL;       //stack to ADL
    assign ADDADL = ctl.ADDADL;     //adder hold register to ADL
    assign DLADH = ctl.DLADH;      //input data latch to high address
    assign ZADH0 = ctl.ZADH0;      //zero ADH0
    assign ZADH1_7 = ctl.ZADH1_7;    //zero ADH1-7
    assign PCHADH = ctl.PCHADH;     //PCH to ADH
    assign SBADH = ctl.SBADH;      //SB to ADH (bidirection pass in schematic)
    assign SSB = ctl.SSB;        //load stack to SB
    assign ADDSB0_6 = ctl.ADDSB0_6;   //ADD to SB
    assign ADDSB7 = ctl.ADDSB7;     //ADD to SB
    assign DBSB = ctl.DBSB;       //DB to SB  (bidirection pass in schematic)
    assign ADHSB = ctl.ADHSB;      //ADH to SB (bidirection pass in schematic)
    assign XSB = ctl.XSB;        //X to SB
    assign YSB = ctl.YSB;        //Y to SB
    assign ACSB = ctl.ACSB;       //accumulator to SB
    assign SS = ctl.SS;         //hold stack
    assign SBS = ctl.SBS;        //load SB (register bus) to stack
    assign ADHABH = ctl.ADHABH;     //ADH to ABH register (load on ph1)
    assign ADLABL = ctl.ADLABL;     //ADL to ABL register (load on ph1)
    assign PCLPCL = ctl.PCLPCL;     //load PCL into PCL select register
    assign ADLPCL = ctl.ADLPCL;     //load ADL into PCL select register
    assign PCHPCH = ctl.PCHPCH;     //load PCH into PCH select register
    assign ADHPCH = ctl.ADHPCH;     //load ADH into PCH select register
    assign IPC = ctl.IPC;        //increment PC
    assign SBAC = ctl.SBAC;       //SB to AC (via decimal adjust logic)
    assign SBX = ctl.SBX;        //SB to X
    assign SBY = ctl.SBY;        //SB to Y
    assign SBADD = ctl.SBADD;      //SB bus
    assign ZADD = ctl.ZADD;       //add zero (not needed??, default ai=0 if SBADD=0) 
    assign INVDBADD = ctl.INVDBADD;   //load inv DB to ALU B
    assign DBADD = ctl.DBADD;      //load DB to ALU B
    assign ADLADD = ctl.ADLADD;     //load ADL to ALU B
    assign IADDC = ctl.IADDC;      //carry in 
    assign DAA = ctl.DAA;         //decimal enable 
    assign SUMS = ctl.SUMS;        //SUM
    assign ANDS = ctl.ANDS;        //AND
    assign EORS = ctl.EORS;        //XOR
    assign ORS = ctl.ORS;         //OR
    assign SRS = ctl.SRS;         //SRR
    assign DSA = ctl.DSA;         //decimal adjust (NA)
    assign DB0C = ctl.DB0C;       //DB0 -> carry bit
    assign IR5C = ctl.IR5C;       //IR5 -> carry bit
    assign ACRC = ctl.ACRC;       //ACR -> carry bit
    assign DB1Z = ctl.DB1Z;       //DB1 -> Z bit
    assign DBZZ = ctl.DBZZ;       //DB=0 -> Z bit
    assign DB2I = ctl.DB2I;       //DB2 -> I bit
    assign IR5I = ctl.IR5I;       //IR5 -> I bit
    assign DB3D = ctl.DB3D;       //DB3 -> D bit      
    assign IR5D = ctl.IR5D;       //IR5 -> D bit
    assign DB6V = ctl.DB6V;       //DB6 -> V bit
    assign AVRV = ctl.AVRV;       //AVR -> V bit
    assign IR5V = ctl.IR5V;       //AVR -> V bit
    assign IV = ctl.IV;         //I bit -> V bit  (???)
    assign DB7N = ctl.DB7N;       //DB7 -> N bit
    assign RW = ctl.RW;
    assign sync = ctl.sync;


endmodule
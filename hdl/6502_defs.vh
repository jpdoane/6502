`ifndef _6502_defs
`define _6502_defs

localparam ASCII_a = 8'h61;

parameter STACKPAGE = 8'h01;

//states
parameter T0      =8'b10000000;
parameter T1      =8'b01000000;
parameter T2      =8'b00100000;
parameter T3      =8'b00010000;
parameter T4      =8'b00001000;
parameter T5      =8'b00000100;
parameter T6      =8'b00000010;
parameter T0T2    =8'b10100000;
parameter T_JAM   =8'b00000000;


parameter OP_BRK      = 5'h00;   // BRK
parameter OP_JSR      = 5'h01;   // JSR
parameter OP_RTI      = 5'h02;   // RTI
parameter OP_RTS      = 5'h03;   // RTS
parameter OP_IMP      = 5'h04;   // impl
parameter OP_IMM      = 5'h05;   // imm
parameter OP_ZPG      = 5'h06;   // zpg
parameter OP_ZXY      = 5'h07;   // zpg X/Y
parameter OP_XIN      = 5'h08;   // X,ind
parameter OP_INY      = 5'h09;   // alu ind,Y ops
parameter OP_ABS      = 5'h0a;   // abs
parameter OP_AXY      = 5'h0b;   // abs, X/Y
parameter OP_PUS      = 5'h0c;   // php,pha
parameter OP_POP      = 5'h0d;   // plp,pha
parameter OP_JUM      = 5'h0e;   // jmp abs
parameter OP_JIN      = 5'h0f;   // jmp ind
parameter OP_BRA      = 5'h10;   // conditional branch
parameter OP_JAM      = 5'h1f;

// parameter T0_EXEC      =6'h33;
// parameter T1_DEBUG      =6'h3f;
// parameter T1_FETCH      =6'h0;

// parameter T2_DECODE     =6'h1;

// parameter T2_ZPG        =6'h2;
// parameter T2_ZPGXY      =6'h3;
// parameter T2_ABSXY      =6'h7;
// parameter T2_ABS        =6'h5;
// parameter T2_XIND       =6'hb;
// parameter T2_INDY       =6'hf;
// parameter T2_JUMP        =6'h17;
// parameter T2_BRANCH     =6'h19;
// parameter T2_PUSH       =6'h1c;
// parameter T2_POP        =6'h1d;
// parameter T2_BRK        =6'h1f;
// parameter T2_RTI        =6'h23;
// parameter T2_RTS        =6'h27;
// parameter T2_JSR        =6'h2b;
// parameter T2_JUMPIND    =6'h2f;

// parameter T3_ZPGXY      =6'h4;
// parameter T3_ABS        =6'h6;
// parameter T3_ABSXY      =6'h8;
// parameter T3_XIND       =6'hc;
// parameter T3_INDY       =6'h10;
// parameter T3_JUMP        =6'h18;
// parameter T3_BRANCH     =6'h1a;
// parameter T3_POP        =6'h1e;
// parameter T3_BRK        =6'h20;
// parameter T3_RTI        =6'h24;
// parameter T3_RTS        =6'h28;
// parameter T3_JSR        =6'h2c;
// parameter T3_JUMPIND    =6'h30;

// parameter T4_ABSXY      =6'h9;
// parameter T4_XIND       =6'hd;
// parameter T4_INDY       =6'h11;
// parameter T4_BRANCH     =6'h1b;
// parameter T4_BRK        =6'h21;
// parameter T4_RTI        =6'h25;
// parameter T4_RTS        =6'h29;
// parameter T4_JSR        =6'h2d;
// parameter T4_JUMPIND    =6'h31;

// parameter T5_XIND       =6'he;
// parameter T5_INDY       =6'h12;
// parameter T5_BRK        =6'h22;
// parameter T5_RTI        =6'h26;
// parameter T5_RTS        =6'h2a;
// parameter T5_JSR        =6'h2e;
// parameter T5_JUMPIND    =6'h32;

// parameter T_RMW_EXEC    =6'h13;
// parameter T_RMW_STORE   =6'h14;
// parameter T_BOOT        =6'h15;


parameter REG_Z = 4'h0;  // 0
parameter RE_NZ = 4'h1;  // -1
parameter REG_D = 4'h2;  // data bus
parameter RE_ND = 4'h3;  // data bus (inverted)
parameter REG_P = 4'h4;  // P status
parameter REG_A = 4'h5;  // accumulator
parameter REG_X = 4'h6;  // X
parameter REG_Y = 4'h7;  // Y
parameter REG_S = 4'h8;  // stack ptr
parameter R_ALU = 4'h9;  // alu register
parameter R_PCL = 4'ha;  // low pc
parameter R_PCH = 4'hb;  // high pc
parameter R_ADL = 4'hc;  // low address
parameter R_ADH = 4'hd;  // high address

parameter ADDR_PC   = 3'h0;
parameter ADDR_DATA   = 3'h1;
parameter ADDR_RES  = 3'h2;
parameter ADDR_ALU  = 3'h3;
parameter ADDR_Z    = 3'h4;
parameter ADDR_INT = 3'h5;
parameter ADDR_STACK = 3'h6;
parameter ADDR_HOLD = 3'h7;


parameter ALU_NOP   =3'h0;
parameter ALU_ADD   =3'h1;
parameter ALU_AND   =3'h2;
parameter ALU_OR    =3'h3;
parameter ALU_XOR   =3'h4;
parameter ALU_SR    =3'h5;
parameter ALU_SL    =3'h6;
parameter ALU_BIT   =3'h7;

parameter IDX_X     =1'b1;
parameter IDX_Y     =1'b0;

parameter FL_N = 8'b10000000;   // Negative
parameter FL_V = 8'b01000000;   // Overflow
parameter FL_U = 8'b00100000;   // Unused, but set on php
parameter FL_B = 8'b00010000;   // Break
parameter FL_D = 8'b00001000;   // Decimal (use BCD for arithmetics)
parameter FL_I = 8'b00000100;   // Interrupt (IRQ disable)
parameter FL_Z = 8'b00000010;   // Zero
parameter FL_C = 8'b00000001;   // Carry
parameter FL_BU= FL_B | FL_U;

`endif
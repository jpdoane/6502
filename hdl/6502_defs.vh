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
parameter T7      =8'b00000001;
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
parameter OP_PUL      = 5'h0d;   // plp,pha
parameter OP_JUM      = 5'h0e;   // jmp abs
parameter OP_JIN      = 5'h0f;   // jmp ind
parameter OP_BRA      = 5'h10;   // conditional branch
parameter OP_JAM      = 5'h1f;

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
`ifndef _6502_defs
`define _6502_defs

localparam ASCII_a = 8'h61;

parameter STACKPAGE = 8'h01;

//states
parameter T0      =10'b0000000001;
parameter T1      =10'b0000000010;
parameter T2      =10'b0000000100;
parameter T3      =10'b0000001000;
parameter T4      =10'b0000010000;
parameter T5      =10'b0000100000;
parameter T6      =10'b0001000000;
parameter T7      =10'b0010000000;
parameter TRMW1   =10'b0100000000;
parameter TRMW2   =10'b1000000000;
parameter TJAM    =10'b0;

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
parameter OP_BRA      = 5'h10;   // conditional branch, taken
parameter OP_BNT      = 5'h11;   // conditional branch, not taken
parameter OP_JAM      = 5'h1f;

parameter ADDR_PC   = 3'h0;
parameter ADDR_DATA   = 3'h1;
parameter ADDR_RES  = 3'h2;
parameter ADDR_ALU  = 3'h3;
parameter ADDR_Z    = 3'h4;
parameter ADDR_INT = 3'h5;
parameter ADDR_STACK = 3'h6;
parameter ADDR_HOLD = 3'h7;


parameter REG_Z =    7'b0000000;
parameter REG_A =    7'b0000001;  // accumulator
parameter REG_X =    7'b0000010;  // X
parameter REG_Y =    7'b0000100;  // Y
parameter REG_S =    7'b0001000;  // stack ptr
parameter REG_ADD =  7'b0010000;  // alu register
parameter REG_D =    7'b0100000;  // sb<=db
parameter REG_ADH =  7'b1000000;   // sb<=adh


parameter STACK_A   = 4'b0001;
parameter STACK_P   = 4'b0010;
parameter STACK_PCL = 4'b0100;
parameter STACK_PCH = 4'b1000;


parameter ALU_NOP   = 5'b00000;
parameter ALU_AND   = 5'b00001;
parameter ALU_ORA   = 5'b00010;
parameter ALU_LSR   = 5'b00100;
parameter ALU_SUM   = 5'b01000;

parameter ALU_CIP   = 5'b10000; // use carry in from p[0], else carry in zero
parameter ALU_ALT   = 5'b00001;         // alt flag for xor, sub, & left sifts
parameter ALU_XOR   = ALU_ORA | ALU_ALT;
parameter ALU_SUB   = ALU_SUM | ALU_ALT; // invert port b to perform subtraction
parameter ALU_ROR   = ALU_LSR | ALU_CIP; // rotate: shift + carry in


// sum-specific flags
parameter ALU_ADZ   = 5'b01010;    // set port b to zero (or -1 with inv)
parameter ALU_CI1   = 5'b01100;    // force carry in=1
parameter ALU_ADC   = ALU_SUM | ALU_CIP; // sum + carry in p[0]
parameter ALU_SBC   = ALU_SUB | ALU_CIP; // sub + carry in p[0]
parameter ALU_CMP   = ALU_SUB | ALU_CI1; // sub + carry in 1
parameter ALU_INC   = ALU_SUM | ALU_ADZ | ALU_CI1; // sum + zero b + carry in 1
parameter ALU_DEC   = ALU_SUB | ALU_ADZ; // sub w/ inv zero


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
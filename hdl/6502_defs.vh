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
parameter T_DEBUG =8'b11111111;

parameter TRMW1   =8'b00010010;
parameter TRMW2   =8'b00001001;

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


parameter REG_Z =    6'b000000;
parameter REG_A =    6'b000001;  // accumulator
parameter REG_X =    6'b000010;  // X
parameter REG_Y =    6'b000100;  // Y
parameter REG_S =    6'b001000;  // stack ptr
parameter REG_ADD =  6'b010000;  // alu register
parameter REG_D =    6'b100000;  // sb<=db


parameter STACK_A   = 4'b0001;
parameter STACK_P   = 4'b0010;
parameter STACK_PCL = 4'b0100;
parameter STACK_PCH = 4'b1000;


parameter ALU_NOP   = 6'b000000;
parameter ALU_AND   = 6'b000001;
parameter ALU_ORA   = 6'b000010;
parameter ALU_SR    = 6'b000100;
parameter ALU_SUM   = 6'b001000;
parameter ALU_CIP   = 6'b010000; // use carry in from p[0], else carry in zero
parameter ALU_OPB   = 6'b100000; // swtich active port to b for unary ops (inc/dec/sr/sl)

parameter ALU_ALT   = 6'b000001;        // alt flag for xor, sub, & left sifts
parameter ALU_XOR   = ALU_ORA | ALU_ALT;
parameter ALU_SUB   = ALU_SUM | ALU_ALT; // invert port b to perform subtraction

// build shift/rot ops
parameter ALU_LSR   = ALU_SR;
parameter ALU_ROR   = ALU_LSR | ALU_CIP;
parameter ALU_ASL   = ALU_LSR | ALU_ALT;
parameter ALU_ROL   = ALU_ROR | ALU_ALT;

// sum-specific flags
parameter ALU_ADZ   = 6'b001010;    // set port b to zero (or -1 with inv)
parameter ALU_CI1   = 6'b001100;    // force carry in=1
parameter ALU_ADC   = ALU_SUM | ALU_CIP; // sum + carry in p[0]
parameter ALU_SBC   = ALU_SUB | ALU_CIP; // sub + carry in p[0]
parameter ALU_CMP   = ALU_SUB | ALU_CI1; // sub + carry in 1
parameter ALU_INC   = ALU_SUM | ALU_ADZ | ALU_CI1; // sum + zero b + carry in 1
parameter ALU_DEC   = ALU_SUB | ALU_ADZ; // sub w/ inv zero
parameter ALU_INB   = ALU_INC | ALU_OPB; // increment db


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
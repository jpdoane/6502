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

parameter ADDR_Z      = 6'b000000;
parameter ADDR_PC     = 6'b000001;
parameter ADDR_DATA   = 6'b000010;
parameter ADDR_ALU    = 6'b000100;
parameter ADDR_INT    = 6'b001000;
parameter ADDR_STACK  = 6'b010000;
parameter ADDR_HOLD   = 6'b100000;


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

// basic ALU ops
parameter ALU_NOP   = 5'b00000;
parameter ALU_AND   = 5'b00001;
parameter ALU_ORA   = 5'b00010;
parameter ALU_XOR   = 5'b00100;
parameter ALU_SHR   = 5'b01000;
parameter ALU_SUM   = 5'b10000;

// ALU flags
parameter ALU_NOF  = 4'b0000;   // 
parameter ALU_CIP  = 4'b0001;   // carry in p[0]
parameter ALU_CI1  = 4'b0010;   // carry in 1
parameter ALU_BIZ  = 4'b0100;   // zero bi
parameter ALU_BIN  = 4'b1000;   // invert bi

parameter ALU_INC  = ALU_BIZ | ALU_CI1; // a + 0
parameter ALU_DEC  = ALU_BIZ | ALU_BIN; // a + -1

// ALU mnumonics: ops + flags
parameter OP_AND = {ALU_NOF, ALU_AND};
parameter OP_ORA = {ALU_NOF, ALU_ORA};
parameter OP_XOR = {ALU_NOF, ALU_XOR};
parameter OP_NOP = {ALU_NOF, ALU_NOP};
parameter OP_ADC = {ALU_CIP, ALU_SUM};
parameter OP_SBC = {ALU_BIN | ALU_CIP, ALU_SUM};
parameter OP_CMP = {ALU_BIN | ALU_CI1, ALU_SUM};
parameter OP_INC = {ALU_INC, ALU_SUM};
parameter OP_DEC = {ALU_DEC, ALU_SUM};
parameter OP_LSR = {ALU_NOF, ALU_SHR};
parameter OP_ROR = {ALU_CIP, ALU_SHR};
parameter OP_ASL = {ALU_NOF, ALU_SUM}; // implemented as M+M
parameter OP_ROL = {ALU_CIP, ALU_SUM}; // implemented as M+M+C

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
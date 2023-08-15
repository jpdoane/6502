`ifndef _6502_defs
`define _6502_defs

localparam ASCII_a = 8'h61;

parameter STACKPAGE = 8'h01;

//states

parameter T0_FETCH      =6'h0;
parameter T1_DECODE     =6'h1;
parameter T2_ZPG        =6'h2;
parameter T2_ZPGXY      =6'h3;
parameter T3_ZPGXY      =6'h4;
parameter T2_ABS        =6'h5;
parameter T3_ABS        =6'h6;
parameter T2_ABSXY      =6'h7;
parameter T3_ABSXY      =6'h8;
parameter T4_ABSXY      =6'h9;
parameter T2_XIND       =6'hb;
parameter T3_XIND       =6'hc;
parameter T4_XIND       =6'hd;
parameter T5_XIND       =6'he;
parameter T2_INDY       =6'hf;
parameter T3_INDY       =6'h10;
parameter T4_INDY       =6'h11;
parameter T5_INDY       =6'h12;
parameter T_RMW_EXEC    =6'h13;
parameter T_RMW_STORE   =6'h14;
parameter T_BOOT        =6'h15;
parameter T_JAM         =6'h16;
parameter T2_JUMP        =6'h17;
parameter T3_JUMP        =6'h18;
parameter T2_BRANCH     =6'h19;
parameter T3_BRANCH     =6'h1a;
parameter T4_BRANCH     =6'h1b;
parameter T2_PUSH       =6'h1c;
parameter T2_POP        =6'h1d;
parameter T3_POP        =6'h1e;
parameter T2_BRK        =6'h1f;
parameter T3_BRK        =6'h20;
parameter T4_BRK        =6'h21;
parameter T5_BRK        =6'h22;
parameter T2_RTI        =6'h23;
parameter T3_RTI        =6'h24;
parameter T4_RTI        =6'h25;
parameter T5_RTI        =6'h26;
parameter T2_RTS        =6'h27;
parameter T3_RTS        =6'h28;
parameter T4_RTS        =6'h29;
parameter T5_RTS        =6'h2a;
parameter T2_JSR        =6'h2b;
parameter T3_JSR        =6'h2c;
parameter T4_JSR        =6'h2d;
parameter T5_JSR        =6'h2e;
parameter T2_JUMPIND    =6'h2f;
parameter T3_JUMPIND    =6'h30;
parameter T4_JUMPIND    =6'h31;
parameter T5_JUMPIND    =6'h32;


parameter REG_Z      =3'h0;
parameter REG_A      =3'h1;
parameter REG_X      =3'h2;
parameter REG_Y      =3'h3;
parameter REG_S      =3'h4;
parameter REG_P      =3'h5;
parameter REG_ADD    =3'h6;
parameter REG_DATA   =3'h7;

parameter DB_Z       =3'h0;
parameter DB_DATA    =3'h1;
parameter DB_PCL     =3'h2;
parameter DB_PCH     =3'h3;
parameter DB_S       =3'h4;
parameter DB_P       =3'h5;
parameter DB_A       =3'h6;
parameter DB_SB      =3'h7;
    
parameter ADDR_PC   = 3'h0;
parameter ADDR_DATA   = 3'h1;
parameter ADDR_RES  = 3'h2;
parameter ADDR_ADD  = 3'h3;
parameter ADDR_Z    = 3'h4;
parameter ADDR_HOLD = 3'h5; 
parameter ADDR_INT = 3'h6;
parameter ADDR_STACK = 3'h7;

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

parameter CARRY_Z   =2'b00;
parameter CARRY_P   =2'b01;
parameter CARRY_C   =2'b10;

parameter FL_NONE = 8'b00000000;   // No flags set
parameter FL_N = 8'b10000000;   // Negative
parameter FL_V = 8'b01000000;   // Overflow
parameter FL_U = 8'b00100000;   // Unused, but set on php
parameter FL_B = 8'b00010000;   // Break
parameter FL_D = 8'b00001000;   // Decimal (use BCD for arithmetics)
parameter FL_I = 8'b00000100;   // Interrupt (IRQ disable)
parameter FL_Z = 8'b00000010;   // Zero
parameter FL_C = 8'b00000001;   // Carry
parameter FL_BU = FL_B | FL_U;  // mask for php, plp 

`endif
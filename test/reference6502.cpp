#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

// #include "perfect6502/types.h"
// #include "perfect6502/netlist_sim.h"
// #include "perfect6502/perfect6502.h"
#include "reference6502.h"


Reference6502::Reference6502(std::string romfile, uint16_t interrupt_port)
: interrupt_port(interrupt_port), cycle_cnt(0)
{
	mem = memory;
	model = initAndResetChip();
	disable_decimal(model);
	loadROM(romfile);
}

Reference6502::~Reference6502()
{
	destroyChip(model);
}


void Reference6502::clock()
{
	if(readclkm1(model) && interrupt_port > 0)
	{
		// feed back writes to interrupt_port to IRQ (bit0) and NMI (bit1) pins
		setIRQ(model, (mem[interrupt_port] >> 0) & 1);
		setNMI(model, (mem[interrupt_port] >> 1) & 1);
	}
	step(model);
}

void Reference6502::cycle()
{
	clock();
	clock();
	cycle_cnt++;
}

void Reference6502::reset()
{
    if(interrupt_port>0) mem[interrupt_port] = 0;
	resetChip(model);
	cycle_cnt = 0;
}

void Reference6502::jump(uint16_t pc)
{
	state6502 state = getState();
	state.pc = pc;
	setState(state);
}

state6502 Reference6502::getState() const
{
    state6502 state;
    state.addr = readAddressBus(model);
    state.pc = readPC(model);
    state.ir = readIR(model);
    state.data = readDataBus(model);
	state.alu = readALU(model);
    state.a = readA(model);
    state.s = readSP(model);
    state.x = readX(model);
    state.y = readY(model);
    state.p = readP(model);
    state.tstate = readTstate(model);
    state.clk = readclkm1(model);
    state.rw = readRW(model);
	state.sync = readSYNC(model);
	state.cycle = cycle_cnt;
    return state;
}

void  Reference6502::setState(const state6502& state)
{
	char memsave[0x200];
	memcpy(memsave, mem, 0x200); //save ZP and stack
	// mini program to init registers.
	uint16_t addr = 0;
	mem[0xFFFC] = 0;	// reset vector to 0x0000
	mem[0xFFFD] = 0;
	mem[addr++] = 0xA9; //lda
	mem[addr++] = state.a;
	mem[addr++] = 0xA2; //ldx
	mem[addr++] = state.s-1;
	mem[addr++] = 0x9A; //txs
	mem[addr++] = 0xA2; //ldx
	mem[addr++] = state.x;
	mem[addr++] = 0xA0; //ldy
	mem[addr++] = state.y;
	mem[0x100 + state.s] = state.p; // store p on stack
	mem[addr++] = 0x28; //plp: pop stack to p
	mem[addr++] = 0x4C; // jump to test program
	mem[addr++] = state.pc & 0xFF;
	mem[addr++] = state.pc >> 8;
	int init_cnt = 26;			// number of clocks to complete

	// run init program
	reset();
	for (int cnt=0; cnt<init_cnt;cnt++)
	{
		cycle();
		// printState(getState());
	}
	memcpy(mem, memsave, 0x200); //restore stack and ZP
	cycle_cnt = 0;
}

bool Reference6502::jammed() const
{
	return false;
    // return readTstate(model) == 0x3f;
}


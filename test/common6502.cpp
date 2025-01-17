#include "common6502.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>

int Abstract6502::loadROM(std::string romfile)
{
    std::ifstream fin(romfile, std::ifstream::binary);
    if (!fin)
	{
        std::cerr << "Error opening file " << romfile << std::endl;
		return 1;
	}

	fin.read((char*) mem, MEMSIZE);

    if (fin.gcount()< MEMSIZE)
        std::cerr << "WARNING: romfile too small " << romfile << ": " << fin.gcount() << " bytes" << std::endl;

	return 0;
}

void printState(const state6502 &state)
{
	char pstr[] = "nv-bdizc";
	for(int i=0; i<8; i++)
		if( i!=5 && (state.p>>i) & 0x1 ) pstr[7-i] -= 32;

	// char tstr[] = "00000000";
	// for(int i=0; i<8; i++)
	// 	tstr[7-i] = (state.tstate>>i) & 0x1 ? '1' : '0';

	printf("cycle %ld: AB:%04X D:%02X%c PC:%04X%c IR:%02X ALU:%02X A:%02X S:%02X X:%02X Y:%02X P:%s(%02X)\n", // T:%s\n",
		   state.cycle,
		   state.addr,
		   state.data, (state.clk && !state.rw) ? '*' : ' ', //print asterisk on write
		   state.pc, state.sync ? '*' : ' ', //print asterisk on sync
		   state.ir,
		   state.alu,
		   state.a,
		   state.s,
		   state.x,
		   state.y,
		   pstr, state.p
		//    ,tstr
		   );
}
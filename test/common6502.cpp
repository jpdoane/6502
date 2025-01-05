#include "common6502.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>

void printState(const state6502 &state)
{
	char pstr[] = "nv-bdizc";
	for(int i=0; i<8; i++)
		if( i!=5 && (state.p>>i) & 0x1 ) pstr[7-i] -= 32;

	char tstr[] = "000000";
	for(int i=0; i<6; i++)
		tstr[5-i] = (state.tstate>>i) & 0x1 ? '1' : '0';

	printf("cycle %ld: AB:%04X D:%02X%c PC:%04X%c IR:%02X ALU:%02X A:%02X S:%02X X:%02X Y:%02X P:%s(%02X) T:%s\n",
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
		   pstr, state.p,
           tstr);
}

std::string emptyLOC(uint16_t pc)
{
	std::stringstream l;
	l << std::hex << pc << "\t\t<no listing>";
	return l.str();
}

void printLOC(uint16_t pc, const std::vector<std::string> listing)
{
	if (pc <= listing.size())
		std::cout << listing[pc] << std::endl;
	else
		std::cout << std::hex << "PC: 0x" << pc << "\t\t<no listing>" << std::endl;
}

void printTrace(const std::vector<state6502> &trace, const std::vector<std::string> &listing, int lines)
{
    if (lines > trace.size())
        lines = trace.size();
    
    for(int i = 0; i<lines; i++)
	{
		state6502 state = trace[trace.size()-lines+i];
		printLOC(state.pc, listing);
		printState(state);
	}
}


int loadROM(uint8_t *mem, std::string romfile, size_t memsize = MEMSIZE )
{
    std::ifstream fin(romfile, std::ifstream::binary);
    if (!fin)
	{
        std::cerr << "Error opening file " << romfile << std::endl;
		return 1;
	}

	fin.read((char*) mem, memsize);

    if (fin.gcount()< memsize)
        std::cerr << "WARNING: romfile too small " << romfile << ": " << fin.gcount() << " bytes" << std::endl;

	return 0;
}

std::vector<std::string> loadListing(std::string listfile)
{
	std::ifstream fin(listfile);
	std::string linestr; 
	std::vector<std::string> listing;

	int pc;
	while (std::getline(fin, linestr))
	{
		try {
			pc = std::stoul(linestr, nullptr, 16);
			if(pc < listing.size())
				listing[pc] = linestr;
			else {
				while(pc >= listing.size())
					listing.push_back(emptyLOC(pc));
				listing.push_back(linestr);
			}
		}
		catch (std::invalid_argument) {} //not all lines in file will be valid LOC
	}
	return listing;
}
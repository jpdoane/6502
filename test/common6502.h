#ifndef COMMON_6502_H
#define COMMON_6502_H

#include <stdint.h>
#include <stdbool.h>
#include <string>
#include <vector>

#define MEMSIZE 0x10000


typedef struct
{
    uint64_t cycle;
    uint16_t addr;
    uint16_t pc;
    uint16_t ir;
    uint8_t data;
    uint8_t alu;
    uint8_t a;
    uint8_t s;
    uint8_t x;
    uint8_t y;
    uint8_t p;
    uint8_t tstate;
    bool clk;
    bool sync;
    bool rw;    
} state6502;

void printState(const state6502 &state);
void printLOC(uint16_t pc, const std::vector<std::string> listing);
void printTrace(const std::vector<state6502> &trace, const std::vector<std::string> &listing, int lines = 10);

int loadROM(uint8_t *mem, std::string romfile, size_t memsize);
std::vector<std::string> loadListing(std::string listfile);


#endif
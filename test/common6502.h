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


class Abstract6502{
protected:
    uint8_t* mem;
public:
    virtual void reset() = 0;
    virtual void jump(uint16_t pc) = 0;
    virtual void setState(const state6502& state) = 0;
    virtual state6502 getState() = 0;
    virtual bool jammed() = 0;
    virtual void cycle() = 0;
    int loadROM(std::string romfile);
};

#endif
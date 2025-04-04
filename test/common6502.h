#ifndef COMMON_6502_H
#define COMMON_6502_H

#include <stdint.h>
#include <stdbool.h>
#include <string>
#include <vector>
#include <iostream>

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

bool operator==(const state6502& lhs, const state6502& rhs);
bool operator!=(const state6502& lhs, const state6502& rhs);

void printState(const state6502 &state, std::ostream& os = std::cout);


class Abstract6502{
public:
    uint8_t* mem;
    virtual ~Abstract6502() {}; // enable destruction of derived class from base ptr

    virtual void reset() = 0;
    virtual void jump(uint16_t pc) = 0;
    virtual void setState(const state6502& state) = 0;
    virtual state6502 getState() const = 0;
    virtual bool jammed() const = 0;
    virtual void cycle() = 0;
    int loadROM(std::string romfile);
};

#endif
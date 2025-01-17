#ifndef REFERENCE_6502_H
#define REFERENCE_6502_H

#include <stdint.h>
#include <stdbool.h>
#include "common6502.h"
// #include "perfect6502/perfect6502.h"

#define state_t void
extern "C" void *initAndResetChip();
extern "C" void destroyChip(state_t *state);
extern "C" void resetChip(state_t *state);
extern "C" void step(state_t *state);
extern "C" unsigned short readPC(state_t *state);
extern "C" unsigned char readA(state_t *state);
extern "C" unsigned char readX(state_t *state);
extern "C" unsigned char readY(state_t *state);
extern "C" unsigned char readSP(state_t *state);
extern "C" unsigned char readP(state_t *state);
extern "C" unsigned int readRW(state_t *state);
extern "C" unsigned char readALU(state_t *state);
extern "C" unsigned char readSB(state_t *state);
extern "C" unsigned short readAddressBus(state_t *state);
extern "C" void writeDataBus(state_t *state, unsigned char);
extern "C" unsigned char readDataBus(state_t *state);
extern "C" unsigned char readIR(state_t *state);
extern "C" unsigned char readTstate(state_t *state);

extern "C" void setNRES(state_t *state, unsigned int nres);
extern "C" void setRDY(state_t *state, unsigned int rdy);
extern "C" void setIRQ(state_t *state, unsigned int irq);
extern "C" void setNMI(state_t *state, unsigned int nmi);
extern "C" void setNSO(state_t *state, unsigned int nso);
extern "C" unsigned int readclkm1(state_t *state);
extern "C" unsigned int readSYNC(state_t *state);
extern "C" void disable_decimal(state_t *state);
extern "C" unsigned char memory[65536];

class Reference6502 : public Abstract6502
{
private:
    state_t* model;
    uint16_t interrupt_port;
    int cycle_cnt;

public:
    Reference6502(std::string romfile, uint16_t interrupt_port = 0);
    ~Reference6502();
    void clock( );
    void cycle( );
    void reset();
    void jump(uint16_t pc);  // will reset chip
    void setState(const state6502& state); 
    state6502 getState();
    bool jammed();
};

#endif

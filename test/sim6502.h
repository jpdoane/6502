#ifndef SIM_6502_H
#define SIM_6502_H

#include <stdint.h>
#include <stdbool.h>
#include <memory>
#include "common6502.h"

#include "build/Vcore_6502.h"
#include "verilated.h"
#include "verilated_fst_c.h"


void stepSim( Vcore_6502* top, VerilatedFstC* tfp=nullptr, bool handlememory=true);
void initSim(int argc, char** argv, VerilatedContext* context, Vcore_6502* top, uint16_t interrupt_port);
void getStateSim(const Vcore_6502* top, state6502* simState);
void resetSim(Vcore_6502* top, const state6502& simState);
void jumpSim(Vcore_6502* top, uint16_t pc);
void closeSim(Vcore_6502* top);

VerilatedFstC* openWaveTrace(Vcore_6502* top, std::string tracefile);
void closeWaveTrace(VerilatedFstC* tfp);
int loadROMSim(std::string romfile);

#endif

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#include "sim6502.h"
#include "verilated.h"
#include "verilated_fst_c.h"

static uint16_t interrupt_port_sim = 0;
static uint8_t memory[MEMSIZE];

void handleMemorySim(Vcore_6502* top)
{
    static uint16_t latch_addr_bus;

    if (top->clk)
    {
        latch_addr_bus = top->addr;    // latch addr to read on 2nd half of cycle
        if(!top->RW) memory[latch_addr_bus] = top->dor;
        top->data_i = 0xff;             // read memory is invalid for 1st half of cycle
        // std::cout << "latching addr " << latch_addr_bus << std:: endl;
    }
    else
    {
        top->data_i = memory[latch_addr_bus];
        // std::cout << std::hex << "read memory[" << latch_addr_bus << "] = " << (int) memory[latch_addr_bus] <<  std:: endl;
    }
}

void stepSimClock( Vcore_6502* top, int nextclk, VerilatedFstC* tfp, bool handlememory)
{
        // interrupt feedback reg
        if(interrupt_port_sim>0 && top->clk){
            top->IRQ = (memory[interrupt_port_sim] >> 0) & 1;
            top->NMI = (memory[interrupt_port_sim] >> 1) & 1;
        }

        top->clk = nextclk;
        top->contextp()->timeInc(1);
        top->eval();
        if(handlememory)
        {
            handleMemorySim(top);
            top->eval();
        }

        if (tfp) tfp->dump(Verilated::time());
}

void stepSim( Vcore_6502* top, VerilatedFstC* tfp, bool handlememory)
{
    stepSimClock(top, 1, tfp, handlememory);
    stepSimClock(top, 0, tfp, handlememory);
}

void initSim(int argc, char** argv, VerilatedContext* context, Vcore_6502* top, uint16_t interrupt_port)
{
    interrupt_port_sim = interrupt_port;

    // initialize Verilator context
    context->debug(0);
    context->randReset(2);
    context->traceEverOn(true); // Verilator must compute traced signals
    context->commandArgs(argc, argv);

    // Set Vcore_6502's input signals and assert reset for a few clocks
    top->READY = 1;
    top->SV = 1;
    top->NMI = 0;
    top->IRQ = 0;
    top->reg_set_en = 0;
    top->rst = 1;  
    top->clk = 1;  
    for (int i=0; i<4; i++) //reset for a few cycles
        stepSim(top);
    top->rst = 0;

}

void resetSim(Vcore_6502* top, const state6502& simState)
{
    // initialize registers
    top->pc_set = simState.pc;
    top->s_set =  simState.s;
    top->a_set =  simState.a;
    top->x_set =  simState.x;
    top->y_set =  simState.y;
    top->p_set =  simState.p;
    top->reg_set_en = 1;
    stepSim( top );
    top->reg_set_en = 0;
    // stepSim( top );
}

void jumpSim(Vcore_6502* top, uint16_t pc)
{
    state6502 mystate;
    getStateSim(top, &mystate);
    mystate.pc = pc;
    resetSim(top, mystate);
}

void getStateSim(const Vcore_6502 *top, state6502* simState)
{
    simState->addr = top->addr;
    simState->pc = top->pc_dbg;
    simState->ir = top->ir_dbg;
    simState->data = top->RW ? top->data_i : top->dor,
	simState->alu = top->alu_dbg;
    simState->a = top->a_dbg;
    simState->s = top->s_dbg;
    simState->x = top->x_dbg;
    simState->y = top->y_dbg;
    simState->p = top->p_dbg;
    simState->tstate = top->tstate_dbg;
    simState->clk = top->clk;
    simState->rw = top->RW;
    simState->sync = top->sync;
    simState->cycle = top->cycle_dbg;
}

void closeSim(std::unique_ptr<Vcore_6502> top)
{
    if(top)
      top->final();
}   

VerilatedFstC* openWaveTrace(Vcore_6502* top, std::string tracefile)
{
    if (tracefile.empty()) return nullptr;
    std::cout << "Logging wavefile " << tracefile << std::endl;
    Verilated::traceEverOn(true);
    VerilatedFstC* tfp = new VerilatedFstC;
    top->trace (tfp, 99);
    tfp->open(tracefile.c_str());
    return tfp;
}   

void closeWaveTrace(VerilatedFstC* tfp)
{
    if(tfp)
        tfp->close();
}   

int loadROMSim(std::string romfile)
{
    int rv = loadROM(memory, romfile, MEMSIZE);
    if(interrupt_port_sim>0) memory[interrupt_port_sim] = 0;
    return rv;
}

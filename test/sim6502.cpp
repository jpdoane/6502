#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#include "sim6502.h"
#include "verilated.h"
#include "verilated_fst_c.h"


Verilated6502::Verilated6502(std::string romfile, uint16_t interrupt_port)
:interrupt_port(interrupt_port), tfp(nullptr), latch_addr_bus(0)
{
    mem = new uint8_t[MEMSIZE];

    // initialize Verilator context
    context = new VerilatedContext;
    context->debug(0);
    context->randReset(2);
    context->traceEverOn(true); // Verilator must compute traced signals
    // context->commandArgs(argc, argv);
    top = new Vcore_6502{context, "core_6502"};

    loadROM(romfile);
    reset();

}

Verilated6502::~Verilated6502()
{
    closeWaveTrace();
    top->final();    
    delete top;
    delete context;
    delete[] mem;
}

void Verilated6502::clock( int clk)
{
        // interrupt feedback reg
        if(interrupt_port>0 && top->clk){
            top->IRQ = (mem[interrupt_port] >> 0) & 1;
            top->NMI = (mem[interrupt_port] >> 1) & 1;
        }

        top->clk = clk;
        top->contextp()->timeInc(1);

        top->eval();

        if (clk)
        {
            latch_addr_bus = top->addr;    // latch addr to read on 2nd half of cycle
            if(!top->RW) mem[latch_addr_bus] = top->dor;
            top->data_i = 0xff;             // read mem is invalid for 1st half of cycle
        }
        else
        {
            top->data_i = mem[latch_addr_bus];
        }

        if (tfp) tfp->dump(Verilated::time());
}

void  Verilated6502::setState(const state6502& state)
{
    // initialize registers
    top->pc_set = state.pc;
    top->s_set =  state.s;
    top->a_set =  state.a;
    top->x_set =  state.x;
    top->y_set =  state.y;
    top->p_set =  state.p;
    top->reg_set_en = 1;
    cycle( );
    top->reg_set_en = 0;
    // stepSim( top );
}

void Verilated6502::reset()
{
    top->READY = 1;
    top->SV = 1;
    top->NMI = 0;
    top->IRQ = 0;
    top->reg_set_en = 0;
    top->rst = 1;  
    top->clk = 1;  

    if(interrupt_port>0) mem[interrupt_port] = 0;

    for (int i=0; i<2; i++)
        cycle();
    top->rst = 0;
}

void Verilated6502::jump(uint16_t pc)
{
    state6502 state = getState();
    state.pc = pc;
    setState(state);
}

state6502 Verilated6502::getState()
{
    state6502 state;
    state.addr = top->addr;
    state.pc = top->pc_dbg;
    state.ir = top->ir_dbg;
    state.data = top->RW ? top->data_i : top->dor,
	state.alu = top->alu_dbg;
    state.a = top->a_dbg;
    state.s = top->s_dbg;
    state.x = top->x_dbg;
    state.y = top->y_dbg;
    state.p = top->p_dbg;
    state.tstate = top->tstate_dbg;
    state.clk = top->clk;
    state.rw = top->RW;
    state.sync = top->sync;
    state.cycle = top->cycle_dbg;
    return state;
}

bool Verilated6502::jammed()
{
    return top->jam;
}


void Verilated6502::openWaveTrace(std::string tracefile)
{
    closeWaveTrace();

    if (tracefile.empty())
        return;

    std::cout << "Logging wavefile " << tracefile << std::endl;
    Verilated::traceEverOn(true);
    VerilatedFstC* tfp = new VerilatedFstC;
    top->trace (tfp, 99);
    tfp->open(tracefile.c_str());
}   

void Verilated6502::closeWaveTrace()
{
    if(tfp){
        tfp->close();
        delete tfp;
        tfp = nullptr;
    }
}   

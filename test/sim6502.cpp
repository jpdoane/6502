#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#include "sim6502.h"

Verilated6502::Verilated6502(std::string romfile, uint16_t interrupt_port)
:interrupt_port(interrupt_port), tfp(nullptr)
{
    mem = new uint8_t[MEMSIZE];

    // initialize Verilator context
    context = new VerilatedContext;
    context->debug(0);
    context->randReset(2);
    // context->commandArgs(argc, argv);
    top = new Vcore_6502{context, "core_6502"};

    if(!romfile.empty())
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

        if (top->rst)
        {
            addr = 0;
            data_o = 0;
            rw = true;
        }
        else if (clk)
        {
            // latch memory bus outputs before rising edge
            addr = top->addr_next;    
            rw = top->RW_next;        
            data_o = top->data_o_next;
            // if (rw) std::cout << std::hex << "before re: rd [" << (int) addr << "] rd " << std::endl;
            // else  std::cout << std::hex << "before re: wr [" << (int) addr << "] <- " << (int) data_o << std::endl;
        }

        top->clk = clk;
        top->contextp()->timeInc(1);
        top->eval();

        if (clk)
        {
            if(!rw) mem[addr] = data_o;
            top->data_i = mem[addr];
            top->eval();

            // if (rw) std::cout << std::hex << "after re: rd [" << (int) addr << "] -> " << (int) mem[addr] << std::endl;
            // else  std::cout << std::hex << "after re: wr [" << (int) addr << "] <- " << (int) mem[addr] << std::endl;
        }
        
        if (tfp) tfp->dump(Verilated::time());         
}

void  Verilated6502::setState(const state6502& state)
{
    // save current reset vector
    uint8_t rvl = mem[0xFFFC];
    uint8_t rvh = mem[0xFFFD];

    // reset chip
    top->READY = 1;
    top->rst = 1;
    cycle( );
    top->rst = 0;
    // set rv to pc
    mem[0xFFFC] = state.pc & 0xff;
    mem[0xFFFD] = state.pc >> 8;

    // wait for reset process to complete
    while(top->core_6502->cycle<6)
        cycle( );

    // restore reset vector
    mem[0xFFFC] = rvl;
    mem[0xFFFD] = rvh;
    cycle( );

    // initialize registers
    top->core_6502->s =  state.s;
    top->core_6502->a =  state.a;
    top->core_6502->x =  state.x;
    top->core_6502->y =  state.y;
    top->core_6502->p =  state.p;
    top->core_6502->cycle =  state.cycle;    

}

void Verilated6502::reset()
{
    top->READY = 1;
    top->SV = 1;
    top->NMI = 0;
    top->IRQ = 0;
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

state6502 Verilated6502::getState() const
{
    state6502 state;
    state.addr = addr;
    state.pc = top->core_6502->pc;
    state.ir = top->core_6502->ir;
    state.data = rw ? top->data_i : data_o,
	state.alu = top->core_6502->add;
    state.a = top->core_6502->a;
    state.s = top->core_6502->s;
    state.x = top->core_6502->x;
    state.y = top->core_6502->y;
    state.p = top->core_6502->p;
    state.tstate = top->core_6502->Tstate;
    state.clk = top->clk;
    state.rw = rw;
    state.sync = top->sync;
    state.cycle = top->core_6502->cycle;
    return state;
}

bool Verilated6502::jammed() const
{
    return top->jam;
}


void Verilated6502::openWaveTrace(std::string tracefile)
{
    closeWaveTrace();

    if (tracefile.empty())
        return;

    context->traceEverOn(true);
    tfp = new VerilatedFstC;
    top->trace (tfp, 99);
    tfp->open(tracefile.c_str());

    if (tfp->isOpen()) {
        std::cout << "Logging to wavefile " << tracefile << std::endl;
    }
    else {
        std::cout << "ERROR opening wavefile " << tracefile << std::endl;
        closeWaveTrace();
    }

}   

void Verilated6502::closeWaveTrace()
{
    if(tfp){
        context->traceEverOn(false);
        tfp->close();
        delete tfp;
        tfp = nullptr;
    }
}   

#include "build/Vcore_6502.h"
#include "verilated.h"
#include "parse_tests.h"
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include "verilated_fst_c.h"

#define Q(x) #x
#define QUOTE(x) Q(x)

void clock_cpu(const std::unique_ptr<VerilatedContext> &context,
                const std::unique_ptr<Vcore_6502> &top,
                std::vector<uint8_t> &ram,
                VerilatedFstC* tfp = nullptr,
                int disable_write=0)
{

        // update any combinatorial changes from C++ code
        top->eval();
        if (tfp) tfp->dump(Verilated::time());

        // clock falling edge
        context->timeInc(1);
        top->clk_m1 = 0;
        top->clk_m2 = ~top->clk_m1;
        top->eval();
        if (tfp) tfp->dump(Verilated::time());

        //register bus
        auto busaddr = top->addr;
        auto rw = top->RW || disable_write;
        auto dread = ram[busaddr];
        auto dwrite = top->dor;

        // clock rising edge
        context->timeInc(1);
        top->clk_m1 = 1;
        top->clk_m2 = ~top->clk_m1;
        top->eval();

        // sim ram
        if( rw )
        {
            top->data_i = dread;
            // std::cout << "reading [" << busaddr << "] -> " << (int) dread << std::endl;
        }
        else
        {
            ram[busaddr] = dwrite;
            // std::cout << "writing [" << busaddr << "] <- " << (int) dwrite << std::endl;
        }

        top->eval();
}

void dump_regs(const std::unique_ptr<Vcore_6502> &top,
                const std::vector<uint8_t> &ram)
{
    std::cout << "REGS: pc=" << (int) top->pc_dbg <<
                ", s=" << (int) top->s_dbg <<
                ", a=" << (int) top->a_dbg <<
                ", x=" << (int) top->x_dbg <<
                ", y=" << (int) top->y_dbg <<
                ", p=" << (int) top->p_dbg << 
                " BUS: " << (top->RW ? "Read: [" : "Write: [" ) << (int) top->addr << "] = " << (int) ram[top->addr] << std::endl;
}

int check_cycle(const std::unique_ptr<Vcore_6502> &top,
                    const std::vector<uint8_t> &ram,
                    const UnitTestCycle &cycle,
                    int cycle_cnt)
{
    int rv=0;

        if(cycle.rw != top->RW)
        {
            std::cerr << "Error: expected " << (cycle.rw ? "read" : "write") <<
                        " but observed " << (top->RW ? "read" : "write") << " in cycle " << cycle_cnt << std::endl;
            rv = 2;
        }
        if(cycle.addr != top->addr)
        {
            std::cerr << "Error: expected bus address " << cycle.addr <<
                        " but observed " << top->addr << " in cycle " << cycle_cnt << std::endl;
            rv = 1;
        }
        if( cycle.data != ram[top->addr])
        {
            std::cerr << "Error: expected bus data " << (int) cycle.data <<
                        " but observed " << (int) ram[top->addr] << " in cycle " << cycle_cnt << std::endl;
            rv = 3;
        }

    if(rv) dump_regs(top, ram);

    return rv;
}

int check_state(    const UnitTestState &state,
                    uint16_t pc, 
                    const std::unique_ptr<Vcore_6502> &top,
                    const std::vector<uint8_t> &ram)
{
    int rv=0;

    if(pc != state.pc) { std::cerr << "Expected pc "<< (int) state.pc << " but observed " << (int) pc << std::endl; rv=4; }
    if(top->s_dbg != state.s) { std::cerr << "Expected s " << (int) state.s << " but observed " <<  (int) top->s_dbg << std::endl; rv=5; }
    if(top->a_dbg != state.a) { std::cerr << "Expected a " << (int) state.a << " but observed " <<  (int) top->a_dbg << std::endl; rv=6; }
    if(top->x_dbg != state.x) { std::cerr << "Expected x " << (int) state.x << " but observed " <<  (int) top->x_dbg << std::endl; rv=7; }
    if(top->y_dbg != state.y) { std::cerr << "Expected y " << (int) state.y << " but observed " <<  (int) top->y_dbg << std::endl; rv=8; }
    if(top->p_dbg != state.p) { std::cerr << "Expected p " << (int) state.p << " but observed " <<  (int) top->p_dbg << std::endl; rv=9; }
    
    for (int i=0; i<state.ram.size(); i++)
    {
        if(ram[state.ram[i].addr] != state.ram[i].data)
            {
                std::cerr << "Expected [" << (int) state.ram[i].addr << "] " << (int) state.ram[i].data <<
                            " but observed " << (int) ram[state.ram[i].addr] << std::endl;
                rv=10;
            }

    }

    return rv;
}

int run_test(const std::unique_ptr<VerilatedContext> &context,
            const std::unique_ptr<Vcore_6502> &top,
            std::vector<uint8_t> &ram,
            const UnitTest &test,
            VerilatedFstC* tfp = nullptr) {

    std::cout << "Test: " << test.name << "...  ";

    // initialize registers
    top->pc_set = test.init.pc;
    top->s_set = test.init.s;
    top->a_set = test.init.a;
    top->x_set = test.init.x;
    top->y_set = test.init.y;
    top->p_set = test.init.p;
    top->reg_set_en = 1;

    // initialize ram
    for (int i=0; i<test.init.ram.size(); i++)
    {
        ram[test.init.ram[i].addr] = test.init.ram[i].data;
        // std::cout << "initializing ram[" << test.init.ram[i].addr << "] <- " << (int) test.init.ram[i].data << std::endl;
    }

    clock_cpu(context, top, ram, tfp, 1);
    top->reg_set_en = 0;


    int rv=0;
    for (int i=0; i<test.cycles.size(); i++) {
        auto cycle = test.cycles[i];
        
        rv += check_cycle(top, ram, cycle, i);
        clock_cpu(context, top, ram, tfp);
    }

    auto pc = top->pc_dbg;
    clock_cpu(context, top, ram, tfp);
    clock_cpu(context, top, ram, tfp);

    rv += check_state(test.fin, pc, top, ram);

    if(rv){
        std::cout << "FAIL!" << std::endl;
        dump_regs(top, ram);
    }
    else
        std::cout << "PASS" << std::endl;
    
    return rv;
}


int main(int argc, char** argv, char** env) {


    // Verilated::mkdir("logs");
    const std::unique_ptr<VerilatedContext> context{new VerilatedContext};
    context->debug(0);
    context->randReset(2);
    context->traceEverOn(true); // Verilator must compute traced signals
    context->commandArgs(argc, argv);

    const std::unique_ptr<Vcore_6502> top{new Vcore_6502{context.get(), "core_6502"}};

    Verilated::traceEverOn(true);
    VerilatedFstC* tfp = new VerilatedFstC;
    top->trace (tfp, 99);
    // tfp->open("unit_test.fst");
    tfp->open(QUOTE(DUMP_WAVE_FILE));

    // Set Vcore_6502's input signals
    top->READY = 1;
    top->SV = 1;
    top->NMI = 0;
    top->IRQ = 0;
    top->reg_set_en = 0;

    std::vector<uint8_t> ram(65536);

    // Assert reset for a few clocks
    top->rst = 1;  
    for (int i=0; i<3; i++) clock_cpu(context, top, ram, tfp);
    // Clear reset for a few clocks (CPU will boot to reset vector)
    top->rst = 0;  
    for (int i=0; i<5; i++) clock_cpu(context, top, ram, tfp);

   
    auto testset = read_testset("./65x02/nes6502/v1/69.json");    
    for(auto test: testset)
        if(run_test(context, top, ram, test, tfp)) break;

    clock_cpu(context, top, ram, tfp);
    clock_cpu(context, top, ram, tfp);

    top->final();
    tfp->close();

    return 0;
}


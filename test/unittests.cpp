#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <getopt.h>

#include "common6502.h"
#include "sim6502.h"
#include "parse_tests.h"


int check_cycle(const std::unique_ptr<Vcore_6502> &top,
                    const std::vector<uint8_t> &ram,
                    const UnitTestCycle &cycle,
                    int cycle_cnt,
                    int verbose=1)
{
    int rv=0;

        if(cycle.rw != top->RW)
        {
            if(verbose) std::cout << std::hex << "Error: expected " << (cycle.rw ? "read" : "write") <<
                        " but observed " << (top->RW ? "read" : "write") << " in cycle " << cycle_cnt << std::endl;
            rv = 2;
        }
        if(cycle.addr != top->addr)
        {
            if(verbose) std::cout << std::hex << "Error: expected bus address " << cycle.addr <<
                        " but observed " << top->addr << " in cycle " << cycle_cnt << std::endl;
            rv = 1;
        }

        uint8_t data = cycle.rw ? ram[top->addr] : top->dor;
        if( cycle.data != data)
        {
            if(verbose) std::cout << std::hex << "Error: expected bus data " << (int) cycle.data <<
                        " but observed " << (int) data << " in cycle " << cycle_cnt << std::endl;
            rv = 3;
        }

    // if(rv) dump_regs(top, ram);

    return rv;
}

int check_state(    const UnitTestState &state,
                    uint16_t pc, 
                    const std::unique_ptr<Vcore_6502> &top,
                    const std::vector<uint8_t> &ram,
                    int verbose=1)
{
    int rv=0;

    if(pc != state.pc)       
        { rv=4;  if(verbose) std::cout << std::hex << "Expected pc="<< (int) state.pc << " but observed " << (int) pc << std::endl; }
    if(top->s_dbg != state.s)
        { rv=5;  if(verbose) std::cout << std::hex << "Expected s=" << (int) state.s << " but observed " <<  (int) top->s_dbg << std::endl; }
    if(top->a_dbg != state.a)
        { rv=6;  if(verbose) std::cout << std::hex << "Expected a=" << (int) state.a << " but observed " <<  (int) top->a_dbg << std::endl; }
    if(top->x_dbg != state.x)
        { rv=7;  if(verbose) std::cout << std::hex << "Expected x=" << (int) state.x << " but observed " <<  (int) top->x_dbg << std::endl; }
    if(top->y_dbg != state.y)
        { rv=8;  if(verbose) std::cout << std::hex << "Expected y=" << (int) state.y << " but observed " <<  (int) top->y_dbg << std::endl; }
    if(top->p_dbg != state.p)
        { rv=9;  if(verbose) std::cout << std::hex << "Expected p=" << (int) state.p << " but observed " <<  (int) top->p_dbg << std::endl; }
    
    for (int i=0; i<state.ram.size(); i++)
    {
        if(ram[state.ram[i].addr] != state.ram[i].data)
            {
                if(verbose) std::cout << std::hex << "Expected [" << (int) state.ram[i].addr << "] " << (int) state.ram[i].data <<
                            " but observed " << (int) ram[state.ram[i].addr] << std::endl;
                rv=10;
            }

    }
    return rv;
}

int run_test(Vcore_6502* top,
                const UnitTest &test,
                int idx,
                int verbose,
                VerilatedFstC* tfp = nullptr) {

    if(verbose) std::cout << "Test " << idx << ": " << test.name << std::endl;
    else std::cout << "Test " << idx << ": " << test.name << "...  ";

    // initialize registers
    state6502 state;
    state.pc = test.init.pc;
    state.s = test.init.s;
    state.a = test.init.a;
    state.x = test.init.x;
    state.y = test.init.y;
    state.p = test.init.p;
    resetSim(top, state);

    // initialize ram
    for (int i=0; i<test.init.ram.size(); i++)
    {
        ram[test.init.ram[i].addr] = test.init.ram[i].data;
        if(verbose) std::cout << "initializing ram[" << test.init.ram[i].addr << "] <- " << (int) test.init.ram[i].data << std::endl;
    }

    clock_cpu(context, top, ram, 0, tfp);
    top->reg_set_en = 0;
    clock_cpu(context, top, ram, 0, tfp);

    int rv=0;
    for (int i=0; i<test.cycles.size(); i++) {
        auto cycle = test.cycles[i];
        
        rv += check_cycle(top, ram, cycle, i);
        clock_cpu(context, top, ram, verbose, tfp, i);
    }

    auto pc = top->pc_dbg;
    clock_cpu(context, top, ram, verbose, tfp, test.cycles.size());
    rv += check_state(test.fin, pc, top, ram);
    clock_cpu(context, top, ram, verbose, tfp, test.cycles.size()+1);

    if(rv){
        if(verbose)
            std::cout << "Test " << idx << ": " << test.name << " FAILED!" << std::endl;
        else
            std::cout << "FAIL!" << std::endl;
        // dump_regs(top, ram);
    }
    else
        std::cout << "PASS" << std::endl;
    
    return rv;
}


int main(int argc, char** argv) {

    char* testfile = NULL;
    for(int i=1; i<argc; i++)
    {
        if(strncmp(argv[i], "+ver", 4) != 0) // all verilator flags start with +verilator...
    		testfile = argv[i];
    }
    if( testfile == NULL)
    {
        std::cerr << "Error - no testfile!" <<std::endl;
        exit(EXIT_FAILURE);
    }

    Vcore_6502* top = makeSim(argc, argv);
    VerilatedFstC* tfp = openWaveTrace(top, wavefile);
    
    // if (startvec_en)
    //     jumpSim(top, startvec);
    

    std::cout << "Loading testfile " << testfile << std::endl;
    auto testset = read_testset(testfile);
    for(std::size_t i = 0; i < testset.size(); ++i)
        if(run_test(context, top, ram, testset[i], i, 0, tfp))
        {
            // run failing test again in verbose mode
            int rv = run_test(context, top, ram, testset[i], i, 1, tfp);
            tfp->close();
            top->final();
            exit(EXIT_FAILURE);
            break;
        }


    closeWaveTrace(tfp);
    closeSim(top);

    return 0;
}


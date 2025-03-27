#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <getopt.h>

#include "common6502.h"
#include "sim6502.h"
#include "reference6502.h"
#include "parse_tests.h"

#define Q(x) #x
#define QUOTE(x) Q(x)


int check_cycle(Abstract6502* sim, const UnitTestCycle &cycle, int cycle_cnt, int verbose=1)
{
    int rv=0;
    state6502 state = sim->getState();

    if(cycle.rw != state.rw)
    {
        if(verbose) std::cout << std::hex << "Error: expected " << (cycle.rw ? "read" : "write") <<
                    " but observed " << (state.rw ? "read" : "write") << " in cycle " << cycle_cnt << std::endl;
        rv = 2;
    }
    if(cycle.addr != state.addr)
    {
        if(verbose) std::cout << std::hex << "Error: expected bus address " << cycle.addr <<
                    " but observed " << state.addr << " in cycle " << cycle_cnt << std::endl;
        rv = 1;
    }

    if( cycle.data != state.data)
    {
        if(verbose) std::cout << std::hex << "Error: expected bus data " << (int) cycle.data <<
                    " but observed " << (int) state.data << " in cycle " << cycle_cnt << std::endl;
        rv = 3;
    }

    return rv;
}
int check_state(Abstract6502* sim, const UnitTestState &test_state, int verbose=1)
{
    int rv=0;
    // check pc on this cycle
    state6502 sim_state = sim->getState();
    if(test_state.pc != sim_state.pc)       
        { rv=4;  if(verbose) std::cout << std::hex << "Expected pc="<< (int) test_state.pc << " but observed " << (int) sim_state.pc << std::endl; }


    for (int i=0; i<test_state.ram.size(); i++)
    {
        int addr = test_state.ram[i].addr;
        int data =test_state.ram[i].data;
        if(sim->mem[addr] != data)
            {
                if(verbose) std::cout << std::hex << "Expected [" << addr << "] " << data <<
                            " but observed " << (int) sim->mem[addr] << std::endl;
                rv=10;
            }

    }
        
    
    // registers need another clock to register
    sim->cycle();
    sim_state = sim->getState();
    if(test_state.s != sim_state.s)
        { rv=5;  if(verbose) std::cout << std::hex << "Expected s=" << (int) test_state.s << " but observed " <<  (int) sim_state.s << std::endl; }
    if(test_state.a != sim_state.a)
        { rv=6;  if(verbose) std::cout << std::hex << "Expected a=" << (int) test_state.a << " but observed " <<  (int) sim_state.a << std::endl; }
    if(test_state.x != sim_state.x)
        { rv=7;  if(verbose) std::cout << std::hex << "Expected x=" << (int) test_state.x << " but observed " <<  (int) sim_state.x << std::endl; }
    if(test_state.y != sim_state.y)
        { rv=8;  if(verbose) std::cout << std::hex << "Expected y=" << (int) test_state.y << " but observed " <<  (int) sim_state.y << std::endl; }
    if(test_state.p != sim_state.p)
        { rv=9;  if(verbose) std::cout << std::hex << "Expected p=" << (int) test_state.p << " but observed " <<  (int) sim_state.p << std::endl; }
    

    return rv;
}

int run_test(Abstract6502* sim, const UnitTest &test, int verbose) {

    if(verbose) std::cout << "Test: " << test.name << std::endl;
    else std::cout << "Test: " << test.name << "...  ";

    // initialize ram
    for (int i=0; i<test.init.ram.size(); i++)
    {
        sim->mem[test.init.ram[i].addr] = test.init.ram[i].data;
        if(verbose) std::cout << std::hex << "initializing ram[" << test.init.ram[i].addr << "] <= " << (int) test.init.ram[i].data << std::endl;
    }

    // initialize registers
    state6502 state;
    state.pc = test.init.pc;
    state.s = test.init.s;
    state.a = test.init.a;
    state.x = test.init.x;
    state.y = test.init.y;
    state.p = test.init.p;
    state.cycle = 0;
    sim->setState(state);

    int rv=0;
    for (int i=0; i<test.cycles.size(); i++) {
        rv += check_cycle(sim, test.cycles[i], i, verbose);
        sim->cycle();
    }

    rv += check_state(sim, test.fin, verbose);
    sim->cycle();
    sim->cycle();

    if(rv){
        if(verbose)
            std::cout << "Test: " << test.name << " FAILED!" << std::endl;
        else
            std::cout << "FAIL!" << std::endl;
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

    Verilated6502* sim = new Verilated6502(std::string());
    
    std::cout << "Loading testfile " << testfile << std::endl;
    auto testset = read_testset(testfile);
    int rv = 0;
    for(std::size_t i = 0; i < testset.size(); ++i)
        if(run_test(sim, testset[i], 0))
        {
            // rerun failing test in verbose mode
            sim->openWaveTrace(QUOTE(DUMP_WAVE_FILE));
            rv = run_test(sim, testset[i], 1);
            break;
        }

    delete sim;

    if(rv)
        exit(EXIT_FAILURE);

    return 0;
}


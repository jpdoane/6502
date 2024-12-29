#include "build/Vcore_6502.h"
#include "verilated.h"
#include "parse_tests.h"
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include "verilated_fst_c.h"

#include <getopt.h>

#define Q(x) #x
#define QUOTE(x) Q(x)


void dump_regs(const std::unique_ptr<Vcore_6502> &top,
                const std::vector<uint8_t> &ram)
{

    char pstr[] = "nv-bdizc";
	uint8_t p = top->p_dbg;
	for(int i=0; i<8; i++)
		if( i!=5 && (p>>i) & 0x1 ) pstr[7-i] -= 32;

	// printf("m1:%d AB:%04X D:%02X PC:%04X IR:%02X SB:%02X ALU:%02X A:%02X S:%02X X:%02X Y:%02X P:%02X(%s)",
	printf("clk1:%d AB:%04X D:%02X PC:%04X A:%02X S:%02X X:%02X Y:%02X P:%02X(%s)",
		   top->clk_m1,
		   top->addr,
		   top->data_i,
		   top->pc_dbg,
		//    readIR(state),
		//    readSB(state),
		//    readALU(state),
		   top->a_dbg,
		   top->s_dbg,
		   top->x_dbg,
		   top->y_dbg,
		   p, pstr);
               
        if (top->clk_m2) {
            if (top->RW)
            printf(" R$%04X=$%02X", top->addr, ram[top->addr]);
            else
            printf(" W$%04X=$%02X", top->addr, top->dor);
        }
        printf("\n");

    // std::cout << std::hex << "REGS: pc=" << (int) top->pc_dbg <<
    //             ", s=" << (int) top->s_dbg <<
    //             ", a=" << (int) top->a_dbg <<
    //             ", x=" << (int) top->x_dbg <<
    //             ", y=" << (int) top->y_dbg <<
    //             ", p=" << (int) top->p_dbg << 
    //             " BUS: " << (top->RW ? "Read: [" : "Write: [" ) << (int) top->addr << "] = " << (int) ram[top->addr] << std::endl;
}

void clock_cpu(const std::unique_ptr<VerilatedContext> &context,
                const std::unique_ptr<Vcore_6502> &top,
                std::vector<uint8_t> &ram,
                int verbose = 0,
                VerilatedFstC* tfp = nullptr,
                int disable_write=0)
{

        // update any combinatorial changes from C++ code
        top->eval();

        // record second half of cycle...
        if(verbose) dump_regs(top, ram);
        if (tfp) tfp->dump(Verilated::time());

        // clock rising edge
        context->timeInc(1);
        top->clk_m1 = 1;
        top->clk_m2 = 0;
        top->eval();


        // capture addr bus state
        auto busaddr = top->addr;
        auto rw = top->RW;
        // auto rw = top->RW || disable_write;
        // auto dread = ram[busaddr];
        // auto dwrite = top->dor;

        top->data_i = 0xff;

        if(rw) {
            // top->data_i = ram[busaddr];
        } else {
            ram[busaddr] = top->dor;
            // top->data_i = top->dor;
        }
        top->eval();

        // record first half of cycle...
        if(verbose) dump_regs(top, ram);
        if (tfp) tfp->dump(Verilated::time());

        // clock falling edge
        context->timeInc(1);
        top->clk_m1 = 0;
        top->clk_m2 = 1;
        top->eval();

        // mem as immediate
        if(rw)
            top->data_i = ram[busaddr];
        else {
            // ram[busaddr] = top->dor;
            top->data_i = top->dor;
        }
        top->eval();
        // to verilator combinatorics....
}

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

int run_test(const std::unique_ptr<VerilatedContext> &context,
            const std::unique_ptr<Vcore_6502> &top,
            std::vector<uint8_t> &ram,
            const UnitTest &test,
            int idx,
            int verbose,
            VerilatedFstC* tfp = nullptr) {

    if(verbose) std::cout << "Test " << idx << ": " << test.name << std::endl;
    else std::cout << "Test " << idx << ": " << test.name << "...  ";
    

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
        if(verbose) std::cout << "initializing ram[" << test.init.ram[i].addr << "] <- " << (int) test.init.ram[i].data << std::endl;
    }

    clock_cpu(context, top, ram, 0, tfp);
    top->reg_set_en = 0;
    clock_cpu(context, top, ram, 0, tfp);

    int rv=0;
    for (int i=0; i<test.cycles.size(); i++) {
        auto cycle = test.cycles[i];
        
        rv += check_cycle(top, ram, cycle, i);
        clock_cpu(context, top, ram, verbose, tfp);
    }

    auto pc = top->pc_dbg;
    clock_cpu(context, top, ram, verbose, tfp);
    rv += check_state(test.fin, pc, top, ram);
    clock_cpu(context, top, ram, verbose, tfp);

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


int main(int argc, char** argv, char** env) {


    // Verilated::mkdir("logs");
    const std::unique_ptr<VerilatedContext> context{new VerilatedContext};
    context->debug(0);
    context->randReset(2);
    context->traceEverOn(true); // Verilator must compute traced signals
    context->commandArgs(argc, argv);

    int opt;
    char* filename = NULL;
    for(int i=1; i<argc; i++)
    {
        if(strncmp(argv[i], "+ver", 4) != 0) // all verilator flags start with +verilator...
        {
    		filename = argv[i];
        }
    }
    if( filename == NULL)
    {
        std::cerr << "Error - no testfile!" <<std::endl;
        exit(EXIT_FAILURE);
    }

    const std::unique_ptr<Vcore_6502> top{new Vcore_6502{context.get(), "core_6502"}};

    // Set Vcore_6502's input signals
    top->READY = 1;
    top->SV = 1;
    top->NMI = 0;
    top->IRQ = 0;
    top->reg_set_en = 0;

    std::vector<uint8_t> ram(65536);

    // Assert reset for a few clocks
    top->rst = 1;  
    for (int i=0; i<3; i++) clock_cpu(context, top, ram, 0);
    // Clear reset for a few clocks (CPU will boot to reset vector)
    top->rst = 0;  
    for (int i=0; i<5; i++) clock_cpu(context, top, ram, 0);

    
    Verilated::traceEverOn(true);
    VerilatedFstC* tfp = new VerilatedFstC;
    top->trace (tfp, 99);
    tfp->open(QUOTE(DUMP_WAVE_FILE));


    std::cout << "Loading testfile " << filename << std::endl;
    auto testset = read_testset(filename);
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

    // clock_cpu(context, top, ram);
    // clock_cpu(context, top, ram);

    tfp->close();
    top->final();

    return 0;
}


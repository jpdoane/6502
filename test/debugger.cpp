#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <getopt.h>
#include <signal.h>

#include "common6502.h"
#include "sim6502.h"
// #include "reference6502.h"

#define BRANCH_OP(ir) ((ir & 0x10) == 0x10)
#define JUMP_OP(ir) ((ir & 0x4c) == 0x4c)

static volatile sig_atomic_t caught_int = 0;
static void intHandler(int) {caught_int = 1;}

// return 1 if we are jumping to same pc as previous
int detect_trap(const std::vector<state6502> &trace)
{
    if (!BRANCH_OP(trace.back().ir) && !JUMP_OP(trace.back().ir)) return 0; //not jump or branch
    if (trace.size() < 2) return 0; // too few entries
    return (trace.end()-1)->pc == (trace.end()-2)->pc;;
}

std::vector<uint16_t> parseBreakpoints(std::string bp_csv )
{
    std::stringstream bplist;
    std::string bpstr;
    std::vector<uint16_t> bps;
    bplist << bp_csv;
    while( bplist.good() ) {
        getline( bplist, bpstr, ',' );
        bps.push_back( std::stoul(bpstr, nullptr, 16) );
    }
    return bps;
}

void debugSim(Vcore_6502* top, 
            bool verbose, std::vector<uint16_t> bps,
            VerilatedFstC* tfp, const std::vector<std::string> &listing,
            bool stepping = false) {

    bool logging = true;
    std::vector<state6502> trace;

    state6502 simState;
    while(true) {
        stepSim(top,tfp);
        getStateSim(top, &simState);

        // user interrupt - break and step
        if(caught_int) stepping = true;
        caught_int = 0;

        bool prtstate = verbose;

        if (simState.sync) // new instruction
        { 
            trace.push_back(simState);

            if(detect_trap(trace))
            {
                printTrace(trace, listing);
                printf("Trap at PC 0x%x\n", top->pc_dbg);
                break;
            }
            else if(std::find(bps.begin(), bps.end(), simState.pc) != bps.end())
            {
                // encountered breakpoint
                printTrace(trace, listing);
                printf("hit breakpoint\n");
                stepping = true;
                logging = true;
            }
            else if (stepping)
                prtstate = true;
        }
        
        if(prtstate)
            printState(simState);

        if(stepping && simState.sync)
        {
            printf("[S]tep, [c]ontinue, or [q]uit\n");
            int cmd = getchar();
            if (cmd != '\n')
            {
                while(getchar() != '\n') {};

                // if (cmd == 's' || cmd == 'S')
                //     stepping = true;

                if (cmd == 'c' || cmd == 'C')
                    stepping = false;

                if (cmd == 'q' || cmd == 'Q')
                    break;
            }
        }

        if(top->jam)
        {
            printf("CPU is jammed!\n");
            printTrace(trace, listing);
            break;
        }
    }
}

int main(int argc, char** argv, char** env) {

    // trap sigint
    signal(SIGINT, intHandler);

    int opt;
    std::string romfile;
    std::string wavefile;
    std::vector<uint16_t> bps;
    std::vector<std::string> listing;

    uint16_t intport = 0;
    uint16_t startvec = 0x0;
    bool startvec_en = false;
    bool verbose = 0;
    bool stepping = false;


    char c;
    opterr = 0;
    while ((c = getopt(argc, argv, "r:b:l:j:i:w:sv")) != -1)
        switch (c)
        {
        case 'r': // rom file
            romfile = optarg;
            break;
        case 'l': // listing file
            listing = loadListing(optarg);
            break;
        case 'j': // startvec
            startvec = std::stoul(optarg, nullptr, 16);
            startvec_en = true;
            break;
        case 'b': // breakpoints, comma delimited
            bps = parseBreakpoints(optarg);
            break;
        case 'v':
            verbose =  true;
            break;
        case 'i':
            intport = std::stoul(optarg, nullptr, 16);
            break;
        case 'w':
            wavefile = optarg;
            break;
        case 's':
            stepping = true;
            break;
        case '?':
            if (optopt == 'b' || optopt == 'l')
                fprintf(stderr, "Option -%c requires an argument.\n", optopt);
            else if (isprint(optopt))
                fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            else
                fprintf(stderr,
                        "Unknown option character `\\x%x'.\n",
                        optopt);
            return 1;
        default:
            abort();
        }

    if( romfile.empty() )
    {
        std::cerr << "Error - no testfile!" <<std::endl;
        exit(EXIT_FAILURE);
    }

    // load rom
    loadROMSim(romfile);


    // initialize Verilator context
    const std::unique_ptr<VerilatedContext> context{new VerilatedContext};
    const std::unique_ptr<Vcore_6502> top{new Vcore_6502{context.get(), "core_6502"}};
    initSim(argc, argv, context.get(), top.get(), intport);
    
    if (startvec_en)
        jumpSim(top.get(), startvec);

    VerilatedFstC* tfp = openWaveTrace(top.get(), wavefile);

    debugSim(top.get(), verbose, bps, tfp, listing, stepping);
    closeWaveTrace(tfp);
    // closeSim(top);
      top->final();

    return 0;
}


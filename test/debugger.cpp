#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <getopt.h>
#include <signal.h>

#include "common6502.h"
#include "sim6502.h"
#include "reference6502.h"

#define BRANCH_OP(ir) ((ir & 0x10) == 0x10)
#define JUMP_OP(ir) ((ir & 0x4c) == 0x4c)

static volatile sig_atomic_t caught_int = 0;
static void intHandler(int) {caught_int = 1;}


int detect_trap(const std::vector<state6502> &history);
std::vector<uint16_t> parseBreakpoints(std::string bp_csv );
void printTrace(const std::vector<state6502>& history, const std::vector<std::string>& listing, int lines = 10);
std::string emptyLOC(uint16_t pc);
void printLOC(uint16_t pc, const std::vector<std::string>& listing);
std::vector<std::string> loadListing(std::string listfile);


void debug(Abstract6502* sim,
            std::vector<uint16_t> bps,
            std::vector<std::string> listing,
            bool verbose,
            bool stepping = false)
{
    state6502 simState;
    std::vector<state6502> history;
    size_t cnt=0;
    while(true) {
        sim->cycle();
        simState = sim->getState();

        // user interrupt - break and step
        if(caught_int) stepping = true;
        caught_int = 0;

        bool prtstate = verbose;

        if (simState.sync) // new instruction
        { 
            cnt++;
            history.push_back(simState);

            if(detect_trap(history))
            {
                printTrace(history, listing);
                printf("Trap at PC 0x%x\n", simState.pc);
                break;
            }
            else if(std::find(bps.begin(), bps.end(), simState.pc) != bps.end())
            {
                // encountered breakpoint
                printTrace(history, listing);
                printf("hit breakpoint\n");
                stepping = true;
            }
            else if (stepping)
                prtstate = true;
            else if (cnt % 1000 == 0)
                printf("processed %ld instructions...\n",cnt);
        }
        
        if(prtstate)
        {
            printLOC(simState.pc, listing);
            printState(simState);
        }

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

        if(sim->jammed())
        {
            printf("CPU is jammed!\n");
            printTrace(history, listing);
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
    bool refdesign = false;

    char c;
    opterr = 0;
    while ((c = getopt(argc, argv, "r:b:l:j:i:w:svp")) != -1)
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
        case 'p':
            refdesign = true;
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
        std::cerr << "Error - no romfile" <<std::endl;
        exit(EXIT_FAILURE);
    }

    Abstract6502* sim;
    if(refdesign)
    {
        sim = new Reference6502(romfile, intport);
    }
    else
    {
        Verilated6502* vsim = new Verilated6502(romfile, intport);
        vsim->openWaveTrace(wavefile);
        sim = vsim;
    }

    if (startvec_en)
        sim->jump(startvec);

    std::cout << "Debugging " << romfile << "..." << std::endl;
    debug(sim, bps, listing, verbose, stepping);

    delete sim;
    return 0;
}



// return 1 if we are jumping to same pc as previous
int detect_trap(const std::vector<state6502> &history)
{
    if (!BRANCH_OP(history.back().ir) && !JUMP_OP(history.back().ir)) return 0; //not jump or branch
    if (history.size() < 2) return 0; // too few entries
    return (history.end()-1)->pc == (history.end()-2)->pc;;
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


void printTrace(const std::vector<state6502>& history, const std::vector<std::string>& listing, int lines)
{
    if (lines > history.size())
        lines = history.size();
    
    for(int i = 0; i<lines; i++)
	{
		state6502 state = history[history.size()-lines+i];
		printLOC(state.pc, listing);
		printState(state);
	}
}

std::string emptyLOC(uint16_t pc)
{
	std::stringstream l;
	l << std::hex << pc << "\t\t<no listing>";
	return l.str();
}

void printLOC(uint16_t pc, const std::vector<std::string>& listing)
{
	if (pc <= listing.size())
		std::cout << listing[pc] << std::endl;
	else
		std::cout << std::hex << "PC: 0x" << pc << "\t\t<no listing>" << std::endl;
}

std::vector<std::string> loadListing(std::string listfile)
{
    std::vector<std::string> listing;
	std::ifstream fin(listfile);
	std::string linestr; 

	int pc;
	while (std::getline(fin, linestr))
	{
		try {
			pc = std::stoul(linestr, nullptr, 16);
			if(pc < listing.size())
				listing[pc] = linestr;
			else {
				while(pc >= listing.size())
					listing.push_back(emptyLOC(pc));
				listing.push_back(linestr);
			}
		}
		catch (std::invalid_argument) {} //not all lines in file will be valid LOC
	}
    return listing;
}
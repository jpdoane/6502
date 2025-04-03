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


void compare_sims(Abstract6502* sim1,
            Abstract6502* sim2,
            std::vector<uint16_t> bps,
            std::vector<std::string> listing,
            bool verbose)
{
    state6502 sim1State;
    state6502 sim2State;
    std::vector<state6502> history1;
    std::vector<state6502> history2;
    size_t cnt=0;
    while(true) {
        cnt++;

        sim1State = sim1->getState();
        sim2State = sim2->getState();

        bool prtstate = verbose;

        
        // user interrupt - break and step
        if(sim1State.cycle > 1 && sim1State != sim2State)
        {
            printTrace(history1, listing);
            std::cout << "States diverge: " << std::endl;
            std::cout << "HW:  ";
            printState(sim1State);
            std::cout << "Ref: ";
            printState(sim2State);

            sim1->cycle();
            sim2->cycle();
            sim1State = sim1->getState();
            sim2State = sim2->getState();
            std::cout << "HW:  ";
            printState(sim1State);
            std::cout << "Ref: ";
            printState(sim2State);

            sim1->cycle();
            sim2->cycle();
            sim1State = sim1->getState();
            sim2State = sim2->getState();
            std::cout << "HW:  ";
            printState(sim1State);
            std::cout << "Ref: ";
            printState(sim2State);

            break;
        }

        caught_int = 0;
        history1.push_back(sim1State);
        history2.push_back(sim2State);

        if (cnt % 1000 == 0)
            printf("processed %ld cycles...\n",cnt);
        
        if(prtstate)
        {
            printLOC(sim1State.pc, listing);
            printState(sim1State);
            printLOC(sim2State.pc, listing);
            printState(sim2State);
        }

        if(sim1->jammed() || sim2->jammed())
        {
            printf("CPU is jammed!\n");
            printTrace(history1, listing);
            break;
        }

        sim1->cycle();
        sim2->cycle();
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
    bool refdesign = false;
    bool compare = false;
    
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
        case 'p':
            refdesign = true;
            break;
        case 'd':
            compare = true;
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

    if( romfile.empty() || romfile[0] == '-' )
    {
        std::cerr << "Error: no romfile" <<std::endl;
        exit(EXIT_FAILURE);
    }

    Verilated6502* vsim = new Verilated6502(romfile, intport);
    vsim->openWaveTrace(wavefile);
    Abstract6502* sim1 = vsim;
    Abstract6502* sim2 = new Reference6502(romfile, intport);

    if (startvec_en)
    {
        sim1->jump(startvec);
        sim2->jump(startvec);
    }

    state6502 initstate = sim1->getState();
    sim1->setState(initstate);
    sim2->setState(initstate);

    std::cout << "Comparing HW and reference sims using ROM  " << romfile << "..." << std::endl;
    compare_sims(sim1, sim2, bps, listing, verbose);

    delete sim1;
    delete sim2;
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
		if(!listing.empty())
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
	if (pc < listing.size())
		std::cout << listing[pc] << std::endl;
	else
		std::cout << std::hex << "PC: 0x" << pc << "\t\t<no listing>" << std::endl;
}

std::vector<std::string> loadListing(std::string listfile)
{
    std::vector<std::string> listing;
	std::ifstream fin(listfile);
	std::string linestr; 

    int pccol = -1;
    int pcwidth = 4;
	while (std::getline(fin, linestr))
	{
        if(linestr.find("ca65") != std::string::npos){
            pccol = 2;
            break;
        }
        if(linestr.find("asm") != std::string::npos){
            pccol = 8;
            break;
        }
	}
    if (pccol<0){
        std::cerr << "WARNING - unknown list file format!" << std::endl;
        return listing;
    }

	std::string pc_listing;
    int pc = 0;
    int pc_next = 0;

    //TODO: lines with no PC should be added to following PC not previous...
    while (std::getline(fin, linestr))
    {
        pc_next = -1;
        try {
            // std::cout << linestr << std::endl;

            // check for valid PC on this line...
            if (linestr.length() >= pccol+pcwidth && 
                linestr.substr(pccol, pcwidth).find_first_not_of("0123456789abcdefABCDEF") == std::string::npos)
                {
                    // convert to hex number
                    // throws if not valid
                    pc_next = std::stoul(linestr.substr(pccol, pcwidth), nullptr, 16);                
                }
        }
        catch (std::invalid_argument) {} //not all lines in file will be valid PCs

        // on new pc
        if( pc_next > pc)
        {
            // found next pc
            // add empty listings to all missing pcs so far...
            while(pc > listing.size())
                listing.push_back(emptyLOC(pc));

            // add this pc
            listing.push_back(pc_listing);
            pc_listing.clear();
            pc = pc_next;
        }

        // log this line to listing for current pc
        if (pc_listing.empty())
            pc_listing = linestr;
        else
            pc_listing += "\n" + linestr;
    }

    return listing;
}
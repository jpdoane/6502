#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "perfect6502/types.h"
#include "perfect6502/netlist_sim.h"
#include "perfect6502/perfect6502.h"
#include "reference6502.h"

static uint16_t interrupt_port_ref;

void* make6502ref(bool decimal_mode, uint16_t interrupt_port)
{
	void *state = initAndResetChip();

	// disable decimal mode
	if (!decimal_mode) disable_decimal(state);

	interrupt_port_ref = interrupt_port;

	return state;
}

int reset6502ref(void *state, const state6502* regs)
{
	setNRES(state, 0); // reset
	stabilizeChip(state);

	// mini program to init registers.
	uint16_t addr = 0;
	memory[0xFFFC] = 0;	// reset vector to 0x0000
	memory[0xFFFD] = 0;
	memory[addr++] = 0xA9; //lda
	memory[addr++] = regs->a;
	memory[addr++] = 0xA2; //ldx
	memory[addr++] = regs->s;
	memory[addr++] = 0x9A; //lxs
	memory[addr++] = 0xA2; //ldx
	memory[addr++] = regs->x;
	memory[addr++] = 0xA0; //ldy
	memory[addr++] = regs->y;
	memory[0x101 + regs->s] = regs->p; // store p on stack
	memory[addr++] = 0x28; //plp: pop stack to p
	memory[addr++] = 0x08; //php: restore stack pointer
	memory[addr++] = 0x4C; // jump to test program
	memory[addr++] = regs->pc & 0xFF;
	memory[addr++] = regs->pc >> 8;
	int init_cnt = 57;			// number of clocks to complete

	/* hold RESET for 8 cycles */
	for (int i = 0; i < 16; i++)
		step(state);

	setNRES(state, 1); // release reset

	// run init program
	for (int cnt=0; cnt<init_cnt;cnt++)
		step(state);

	return 0;
}

void step6502ref(void *state)
{
	if(readclkm1(state) && interrupt_port_ref > 0)
	{
		// feed back writes to interrupt_port to IRQ (bit0) and NMI (bit1) pins
		setIRQ(state, (memory[interrupt_port_ref] >> 0) & 1);
		setNMI(state, (memory[interrupt_port_ref] >> 1) & 1);
	}

	step(state);
}

void getState6502ref(void *state, state6502* regs)
{
    regs->addr = readAddressBus(state);
    regs->pc = readPC(state);
    regs->ir = readIR(state);
    regs->data = readDataBus(state);
	regs->alu = readALU(state);
    regs->a = readA(state);
    regs->s = readSP(state);
    regs->x = readX(state);
    regs->y = readY(state);
    regs->p = readP(state);
    regs->tstate = readTstate(state);
    regs->clk = readclkm1(state);
    regs->rw = readRW(state);
	regs->sync = readSYNC(state);
}

// void dumpRegsref(void *state)
// {
// 	if (state == NULL) return;

// 	bool clk1 = readclkm1(state);
// 	bool clk2 = readclkm2(state);

// 	uint16_t a = readAddressBus(state);
// 	uint8_t d = readDataBus(state);
// 	bool rw = readRW(state);

// 	char pstr[] = "nv-bdizc";
// 	uint8_t p = readP(state);
// 	for(int i=0; i<8; i++)
// 		if( i!=5 && (p>>i) & 0x1 ) pstr[7-i] -= 32;

// 	char tstr[] = "000000";
// 	uint8_t tstate = readTstate(state);
// 	for(int i=0; i<6; i++)
// 		tstr[5-i] = (tstate>>i) & 0x1 ? '1' : '0';

// 	uint8_t t = readTstate(state);
// 	printf("cyc:%d,%d AB:%04X D:%02X PC:%04X IR:%02X SB:%02X ALU:%02X A:%02X S:%02X X:%02X Y:%02X P:%02X(%s) T:%01d%01d%01d%01d%01d%01d",
// 		   cycle,clk1,
// 		   a,
// 		   d,
// 		   readPC(state),
// 		   readIR(state),
// 		   readSB(state),
// 		   readALU(state),
// 		   readA(state),
// 		   readSP(state),
// 		   readX(state),
// 		   readY(state),
// 		   p, pstr,
// 		   tstr);

// 	if (clk2) {
// 		if (rw)
// 		printf(" R$%04X=$%02X", a, memory[a]);
// 		else
// 		printf(" W$%04X=$%02X", a, d);
// 	}
// 	printf("\n");
// }


// int
// main(int argc, char *argv[])
// {
// 	int opt;
// 	int cycles = 100;
// 	int init_pc=0;
// 	int init_x=0;
// 	int init_y=0;
// 	int init_s=0;
// 	int init_p=0;
// 	int init_a=0;
// 	int h=0;
// 	int d=0;
// 	int intport = 0;
// 	int intport_en = 0;

// 	while((opt = getopt(argc, argv, "hdc:j:a:s:x:y:p:")) != -1)
// 	{
//     switch (opt)
//       {
//       case 'h': //hex mode
//         h = 1;
//         break;
//       case 'd': //disable decimal
//         d = 1;
//         break;
//       case 'c':
//         cycles = atoi(optarg);
//         break;
//       case 'j':
//         init_pc = h ? strtol(optarg, NULL, 16) : atoi(optarg);
//         break;
//       case 'a':
//         init_a = h ? strtol(optarg, NULL, 16) : atoi(optarg);
//         break;
//       case 's':
//         init_s = h ? strtol(optarg, NULL, 16) : atoi(optarg);
//         break;
//       case 'x':
//         init_x = h ? strtol(optarg, NULL, 16) : atoi(optarg);
//         break;
//       case 'y':
//         init_y = h ? strtol(optarg, NULL, 16) : atoi(optarg);
//         break;
//       case 'p':
//         init_p = h ? strtol(optarg, NULL, 16) : atoi(optarg);
//         break;
//       case 'i':
//         intport = h ? strtol(optarg, NULL, 16) : atoi(optarg);
// 		intport_en = 0;
//         break;
//       case '?':
// 		fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
//         return 1;
//       default:
//         exit(EXIT_FAILURE);
//       }
// 	}


// 	FILE *f;
// 	char* filename = NULL;
// 	if(argc > optind){
// 		filename = argv[optind];
// 	}

// 	int clk = 0; 
// 	void *state = initAndResetChip();

// 	// disable decimal mode
// 	if (d)
// 		disable_decimal(state);

// 	int init_cnt;
// 	// init_cnt = 16;

// 	// perform reset and run mini program to init registers.
// 	memory[0xFFFC] = 0;	// reset vector to 0x0000
// 	memory[0xFFFD] = 0;
// 	uint16_t addr = 0;
// 	memory[addr++] = 0xA9; //lda
// 	memory[addr++] = init_a;
// 	memory[addr++] = 0xA2; //ldx
// 	memory[addr++] = init_s;
// 	memory[addr++] = 0x9A; //lxs
// 	memory[addr++] = 0xA2; //ldx
// 	memory[addr++] = init_x;
// 	memory[addr++] = 0xA0; //ldy
// 	memory[addr++] = init_y;
// 	memory[0x101 + init_s] = init_p; // store p on stack
// 	memory[addr++] = 0x28; //plp: pop stack to p
// 	memory[addr++] = 0x08; //php: restore stack pointer
// 	memory[addr++] = 0x4C; // jump to test program
// 	memory[addr++] = init_pc & 0xFF;
// 	memory[addr++] = init_pc >> 8;
// 	init_cnt = 57;

// 	int cnt;
// 	for (cnt=0; cnt<init_cnt;cnt++) {
// 		step(state);
// 		clk = !clk;
// 	};
// 	// printf("initialization complete\n");

// 	if(filename) {
// 		f = fopen(filename, "r");
// 		if(f)
// 		{
// 			// printf("loading ramfile %s.\n", filename);
// 			fread(memory, 1, 65536, f);
// 			fclose(f);
// 		}
// 		else
// 		{
// 			fprintf(stderr, "invalid file: %s\n", filename);
// 			exit(EXIT_FAILURE);
// 		}
// 	}
// 	else{
// 		fprintf(stderr, "warning - no test program!\n");
// 	}




// 	exit(EXIT_SUCCESS);
// }

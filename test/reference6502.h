#ifndef REFERENCE_6502_H
#define REFERENCE_6502_H

#include <stdint.h>
#include <stdbool.h>
#include "common6502.h"

void* make6502ref(bool decimal_mode, uint16_t interrupt_port);
void reset6502ref(void *state, const state6502* regs);
void step6502ref(void *state);
void getState6502ref(void *state, state6502* regs);

#endif

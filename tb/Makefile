include ../inc/make.inc

TOP=cpu_tb
ASM_SOURCE = test6502.s

include ../inc/make.build

HDL_SOURCE = $(CPU_SOURCE) $(TOP).sv
HDL_INCLUDE = $(CPU_INCLUDE)
CUSTOMSIMARGS=-D'ROM_FILE="$(OBJFILE)"' \
			  -DRESET_RAM=1 \
			  -DDEBUG_CPU=1
SIMREQS=$(OBJFILE)

include ../inc/make.sim

 .DEFAULT_GOAL = sim
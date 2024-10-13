include ../inc/make.inc

TOP=core_6502
VERILATE_TOP=test_main.cpp
VERILATE_INCL=input.vc
HDL_SOURCE = $(CPU_SOURCE)

include ../inc/make.verilator

VERILATOR_FLAGS += -DDEBUG_REG=1

 .DEFAULT_GOAL = run
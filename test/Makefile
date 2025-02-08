include ../inc/make.inc

TOP=core_6502
VERILATE_TOP=
HDL_SOURCE = $(CPU_SOURCE)
VERILATOR_FLAGS += -DDEBUG_REG=1

DEBUG_SRC=debugger.cpp common6502.cpp sim6502.cpp reference6502.cpp
DEBUG_TARGET=$(BUILD_PATH)/debugger

PERFECT_PATH=perfect6502
PERFECT_OBJS=$(BUILD_PATH)/perfect6502.o $(BUILD_PATH)/netlist_sim.o

include ../inc/make.verilator

$(BUILD_PATH)/%.o: $(PERFECT_PATH)/%.c
	-@mkdir $(BUILD_PATH)
	gcc -c $^ -o $@


.PHONY: debugger
debugger: $(DEBUG_TARGET)

$(DEBUG_TARGET): $(DEBUG_SRC) $(PERFECT_OBJS) $(VMODEL)
	g++ $^ $(CCFLAGS) -g -o $@

.PHONY: unittests
unittests:
	make -C unittests

.PHONY: testrom
testrom: $(DEBUG_TARGET)
	export TESTFLAGS="-w $(WAVEFILE)"
	export DEBUGGER=$(DEBUG_TARGET)
	make -C testroms functional_test 

.DEFAULT_GOAL = unittests
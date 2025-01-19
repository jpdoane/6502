include ../inc/make.inc

TOP=core_6502
VERILATE_TOP=
VERILATE_INCL=input.vc
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

$(DEBUG_TARGET): $(DEBUG_SRC) $(PERFECT_OBJS) $(VMODEL)
	g++ $^ $(VFLAGS) -g -o $(DEBUG_TARGET)

TESTROM = testroms/build/6502_functional_test.bin
.PHONY: test
test: $(DEBUG_TARGET) $(TESTROM)
	$(DEBUG_TARGET) -r $(TESTROM) -l $(TESTROM:%.bin=%.lst) -w $(WAVEFILE) -j 400 $(TESTFLAGS)

# ALLTESTROM = testroms/build/AllSuiteA.bin
# .PHONY: alltest
# alltest: $(DEBUG_TARGET) $(ALLTESTROM)
# 	$(DEBUG_TARGET) -r $(ALLTESTROM) -j 4000 $(TESTFLAGS)

INTTESTROM = testroms/build/6502_interrupt_test.bin
.PHONY: testint
testint: $(DEBUG_TARGET) $(INTTESTROM)
	$(DEBUG_TARGET) -r $(INTTESTROM) -l $(INTTESTROM:%.bin=%.lst) -i bffc -j 400

.DEFAULT_GOAL = $(DEBUG_TARGET)
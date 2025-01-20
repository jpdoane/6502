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

$(DEBUG_TARGET): $(DEBUG_SRC) $(PERFECT_OBJS) $(VMODEL)
	g++ $^ $(CCFLAGS) -g -o $@


.PHONY: test_rom
test_rom: $(TESTROM) $(DEBUG_TARGET)
	$(DEBUG_TARGET) -r $(TESTROM) -l $(TESTROM:%.bin=%.lst)  $(_TESTFLAGS) $(TESTFLAGS)

.PHONY: functional_test
functional_test: TESTROM=testroms/build/6502_functional_test.bin
functional_test: _TESTFLAGS+=-j 400
functional_test: test_rom

.PHONY: interrupt_test
interrupt_test: TESTROM=testroms/build/6502_interrupt_test.bin 
interrupt_test: _TESTFLAGS+=-j 400
interrupt_test: _TESTFLAGS+=-i bffc
interrupt_test: test_rom


.PHONY: interrupt_test
interrupt_test: TESTROM=testroms/build/6502_interrupt_test.bin 
interrupt_test: _TESTFLAGS+=-j 400
interrupt_test: _TESTFLAGS+=-i bffc
interrupt_test: test_rom

.PHONY: AllSuiteA
AllSuiteA: TESTROM=testroms/build/AllSuiteA.bin 
AllSuiteA: _TESTFLAGS+=-j 4000 -w $(WAVEFILE)
AllSuiteA: test_rom

# ALLTESTROM = testroms/build/AllSuiteA.bin
# .PHONY: alltest
# alltest: $(DEBUG_TARGET) $(ALLTESTROM)
# 	$(DEBUG_TARGET) -r $(ALLTESTROM) -j 4000 $(TESTFLAGS)

.DEFAULT_GOAL = functional_test
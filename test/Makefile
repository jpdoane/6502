include ../inc/make.inc

TOP=core
VERILATE_TOP=
HDL_SOURCE = $(CPU_SOURCE)
VERILATOR_FLAGS += -DDEBUG_REG=1

DEBUG_SRC=debugger.cpp common6502.cpp sim6502.cpp reference6502.cpp
DEBUG_TARGET=$(BUILD_PATH)/debugger

COMPARE_SRC=compare.cpp common6502.cpp sim6502.cpp reference6502.cpp
COMPARE_TARGET=$(BUILD_PATH)/comparer

PERFECT_PATH=perfect6502
PERFECT_OBJS=$(BUILD_PATH)/perfect6502.o $(BUILD_PATH)/netlist_sim.o

include ../inc/make.verilator

$(BUILD_PATH)/%.o: $(PERFECT_PATH)/%.c
	-@mkdir $(BUILD_PATH)
	gcc -c $^ -o $@

.PHONY: debugger
debugger: $(DEBUG_TARGET)

.PHONY: compare
compare: $(COMPARE_TARGET)

$(DEBUG_TARGET): $(DEBUG_SRC) $(PERFECT_OBJS) $(VMODEL)
	g++ $^ $(CCFLAGS) -g -o $@

$(COMPARE_TARGET): $(COMPARE_SRC) $(PERFECT_OBJS) $(VMODEL)
	g++ $^ $(CCFLAGS) -g -o $@

.PHONY: testrom
testrom: $(DEBUG_TARGET)
	make -C testroms functional_test DEBUGGER=$(DEBUG_TARGET)

.PHONY: testrom
compare_testrom: $(COMPARE_TARGET)
	make -C testroms functional_test DEBUGGER=$(COMPARE_TARGET)

include make.unittests
.DEFAULT_GOAL = unittests

clean:
	-rm -rf $(BUILD_PATH)
	-make -C testroms clean 

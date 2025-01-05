include ../inc/make.inc

TOP=core_6502
VERILATE_TOP=sim6502.cpp common6502.cpp
VERILATE_INCL=input.vc
HDL_SOURCE = $(CPU_SOURCE)
VERILATOR_FLAGS += -DDEBUG_REG=1

DEBUG_SRC=debugger.cpp
DEBUG_TARGET=$(BUILD_PATH)/debugger

include ../inc/make.verilator

$(DEBUG_TARGET):  $(DEBUG_SRC) $(VMODEL)
	g++ $^ $(VFLAGS) -g -o $(DEBUG_TARGET)

TESTROM = testroms/build/6502_functional_test.bin
.PHONY: test
test: $(DEBUG_TARGET) $(TESTROM)
	$(DEBUG_TARGET) -r $(TESTROM) -l $(TESTROM:%.bin=%.lst) -j 400


#$(DEBUG_TARGET) -r $(TESTROM) -l $(TESTROM:%.bin=%.lst) -j 400 -w $(WAVEFILE) 

INTTESTROM = testroms/build/6502_interrupt_test.bin
.PHONY: testint
testint: $(DEBUG_TARGET) $(INTTESTROM)
	$(DEBUG_TARGET) -r $(INTTESTROM) -l $(INTTESTROM:%.bin=%.lst) -i bffc -j 400 -w $(WAVEFILE) 

.DEFAULT_GOAL = $(DEBUG_TARGET)
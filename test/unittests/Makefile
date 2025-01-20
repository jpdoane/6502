include ../../inc/make.inc

TOP=core_6502
VERILATE_TOP=
VERILATE_INCL=input.vc
HDL_SOURCE = $(CPU_SOURCE)
CCFLAGS += -I../
# VERILATOR_FLAGS += -DDEBUG_REG=1

TEST_SRC=unittests.cpp ../common6502.cpp ../sim6502.cpp
TEST_TARGET=$(BUILD_PATH)/unittester
TESTPATH=$(abspath .)/65x02/nes6502/v1

testfiles=$(wildcard $(TESTPATH)/*.json)
logfiles=$(patsubst $(TESTPATH)/%.json,$(LOG_PATH)/%.log,$(testfiles))
logsummary=$(LOG_PATH)/summary.log
testops=$(patsubst $(TESTPATH)/%.json,%,$(testfiles))


# PERFECT_PATH=perfect6502
# PERFECT_OBJS=$(BUILD_PATH)/perfect6502.o $(BUILD_PATH)/netlist_sim.o

include ../../inc/make.verilator

# $(BUILD_PATH)/%.o: $(PERFECT_PATH)/%.c
# 	-@mkdir $(BUILD_PATH)
# 	gcc -c $^ -o $@

$(TEST_TARGET): $(TEST_SRC) $(VMODEL)
	g++ $^ $(CCFLAGS) -g -o $@

.PHONY: all
all: $(logfiles)

# uncomment to force retesting...
.PHONY: $(logfiles)
$(logfiles): $(LOG_PATH)/%.log: $(TESTPATH)/%.json $(TEST_TARGET)
	$(TEST_TARGET) $< | tee $@; exit $${PIPESTATUS[0]}

.PHONY: $(testops)
$(testops): %: $(LOG_PATH)/%.log

imm_impl: 09 29 49 69 e9 9a ba
zp: 24 84 a4 c4 e4 05 25 45 65 85 a5 c5 e5
zp_xy: 94 b4 15 35 55 75 95 b5 d5 f5 
abs: 2c 8c ac cc ec 0d 2d 4d 6d 8d cd ed 
abs_xy: bc 1d 3d 5d 7d 9d bd dd fd 19 39 59 79 99 b9 d9 f9 be
ind_x: 01 21 41 61 81 a1 c1 e1
ind_y: 11 31 51 71 91 b1 d1 f1
rmw: 0a 2a 4a 6a 8a aa ca ea 06 26 46 66 86 a6 c6 e6 0e 2e 4e 6e 8e ae ce ee 16 36 56 76 96 b6 d6 f6 96 b6 1e 3e 5e 7e de fe
stack: 08 48 28 68
br: 10 30 50 70 90 b0 d0 f0
jmp: 4c 6c
call_break: 00 20 40 60

valid: imm_impl zp zp_xy abs abs_xy ind_x ind_y rmw stack br jmp call_break
illegal: 80 02 12 22 32 42 52 62 72 82 92 b2 c2 d2 e2 f2 03 13 23 33 43 53 63 73 83 93 a3 b3 c3 d3 e3 f3 04 14 34 44 54 64 74 d4 f4 07 17 27 37 47 57 67 77 87 97 a7 b7 c7 d7 e7 f7 1a 3a 5a 7a da fa 0b 1b 2b 3b 4b 5b 6b 7b 8b 9b ab bb cb db eb fb 0c 1c 3c 5c 7c 9c dc fc 9e 0f 1f 2f 3f 4f 5f 6f 7f 8f 9f af bf cf df ef ff 


wip: 

.DEFAULT_GOAL = valid

include ../inc/make.inc

TOP=core_6502
VERILATE_TOP=test_main.cpp
VERILATE_INCL=input.vc
HDL_SOURCE = $(CPU_SOURCE)
VERILATOR_FLAGS += -DDEBUG_REG=1

include ../inc/make.verilator

TESTPATH=$(abspath .)/65x02/nes6502/v1


testfiles=$(wildcard $(TESTPATH)/*.json)
logfiles=$(patsubst $(TESTPATH)/%.json,$(LOG_PATH)/%.log,$(testfiles))
logsummary=$(LOG_PATH)/summary.log
testops=$(patsubst $(TESTPATH)/%.json,%,$(testfiles))


# .PHONY: clearsummary
# clearsummary:
# 	-rm $(logsummary)

.PHONY: all
all: $(logfiles)

# uncomment to force retesting...
.PHONY: $(logfiles)
$(logfiles): $(LOG_PATH)/%.log: $(TESTPATH)/%.json $(TARGET)
	$(TARGET) $< | tee $@; exit $${PIPESTATUS[0]}

.PHONY: $(testops)
$(testops): %: $(LOG_PATH)/%.log

imm_impl: 09 29 49 69 e9 0a 2a 4a 6a 8a aa ca ea
zp: 24 84 a4 c4 e4 05 25 45 65 85 a5 c5 e5
zp_xy: 94 b4 15 35 55 75 95 b5 d5 f5 16 36 56 76 96 b6 d6 f6 96 b6
abs: 2c 8c ac cc ec 0d 2d 4d 6d 8d cd ed 0e 2e 4e 6e 8e ae ce ee
abs_xy: bc 1d 3d 5d 7d 9d bd dd fd 1e 3e 5e 7e be de fe 19 39 59 79 99 b9 d9 f9
ind_x: 01 21 41 61 81 a1 c1 e1
ind_y: 11 31 51 71 91 b1 d1 f1
stack: 08 48 28 68
br: 10 30 50 70 90 b0 d0 f0
jmp: 4c 6c
call_break: 00 20 40 60

passing: imm_impl zp zp_xy abs abs_xy ind_x ind_y br jmp stack
wip: call_break

.DEFAULT_GOAL = wip

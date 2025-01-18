# Makefile

# defaults
SIM ?= verilator
WAVES=1
# EXTRA_ARGS += --trace --trace-fst --trace-structs
TOPLEVEL_LANG ?= verilog

HDL_PATH=$(PWD)/../hdl
VERILOG_INCLUDE_DIRS=$(PWD)/../hdl

VERILOG_SOURCES = $(HDL_PATH)/core_6502.sv
# use VHDL_SOURCES for VHDL files

# TOPLEVEL is the name of the toplevel module in your Verilog or VHDL file
TOPLEVEL = core_6502

# MODULE is the basename of the Python test file
MODULE = test_core_6502

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim

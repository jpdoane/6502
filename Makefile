HDLDIR=hdl
SIMDIR=sim
ASMDIR=src
BUILDDIR=build
TBDIR=testbench

HDLEXT=sv
ASMEXT=s
SIMEXT=vvp
WAVEEXT=vcd
ROMEXT=o

HDLSOURCES = $(wildcard $(HDLDIR)/*.$(HDLEXT))
HDLTESTBENCH=$(TBDIR)/cpu_tb.sv

OBJTARGETNAME=test
OBJTARGET=$(BUILDDIR)/$(OBJTARGETNAME).$(ROMEXT)
SIMTARGET=$(BUILDDIR)/$(OBJTARGETNAME).$(SIMEXT)
WAVETARGET=$(BUILDDIR)/$(OBJTARGETNAME).$(WAVEEXT)
OBJDUMP=$(BUILDDIR)/$(OBJTARGETNAME).dump

.PHONY: all
all: $(WAVETARGET)

.PHONY: view
view: $(WAVETARGET)
	gtkwave $^ -a $(SIMDIR)/$(OBJTARGETNAME).sav &

# compute 6502 obj from asm
$(BUILDDIR)/%.$(ROMEXT): $(ASMDIR)/%.$(ASMEXT)
	@mkdir -p $(@D)
	xa -o $@ $^ 

# compute verilog sim
$(SIMTARGET): $(HDLTESTBENCH) $(HDLSOURCES) $(OBJTARGET)
	@mkdir -p $(@D)
	iverilog -g2012 -o $@ \
				-D'DUMP_WAVE_FILE="$(WAVETARGET)"' \
				-D'ROM_FILE="$(OBJTARGET)"' \
				-I $(HDLDIR) $(HDLSOURCES) $<

# run verilog sim
$(WAVETARGET): $(SIMTARGET)
	vvp $^

.PHONY: clean
clean:
	rm -rf $(BUILDDIR)
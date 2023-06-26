HDLDIR=hdl
SIMDIR=sim
ASMDIR=src
BUILDDIR=build
TBDIR=testbench

HDLEXT=sv
HDLINCEXT=svi
ASMEXT=s
SIMEXT=vvp
WAVEEXT=vcd
ROMEXT=o

HDLSOURCES = $(wildcard $(HDLDIR)/*.$(HDLEXT)) $(wildcard $(HDLDIR)/*.$(HDLINCEXT))
HDLTESTBENCH=$(TBDIR)/cpu_tb.sv

ASMSOURCE =  $(wildcard $(ASMDIR)/*.$(ASMEXT))
ASMOBJ = $(patsubst $(ASMDIR)%,$(BUILDDIR)%,$(ASMSOURCE:.$(ASMEXT)=.$(ROMEXT)))
WAVEOBJ = $(patsubst %.$(ROMEXT),%.$(WAVEEXT),$(ASMOBJ))
WAVESAVEFILE=$(SIMDIR)/test.sav

.PHONY: all
all: $(WAVEOBJ)

.PHONY: view
view: $(WAVETARGET)
	gtkwave $^ -a $(WAVESAVEFILE) &

# compute 6502 obj from asm
$(BUILDDIR)/%.$(ROMEXT): $(ASMDIR)/%.$(ASMEXT)
	@mkdir -p $(@D)
	xa -o $@ $^ 

# compute verilog sim
$(BUILDDIR)/%.$(SIMEXT): $(BUILDDIR)/%.$(ROMEXT) $(HDLTESTBENCH) $(HDLSOURCES)
	iverilog -g2012 -o $@ \
				-D'DUMP_WAVE_FILE="$(patsubst %.o,%.vcd,$<)"' \
				-D'ROM_FILE="$<"' \
				-I $(HDLDIR) $(HDLSOURCES) $(HDLTESTBENCH)

# run verilog sim
$(BUILDDIR)/%.$(WAVEEXT): $(BUILDDIR)/%.$(SIMEXT)
	vvp $^

.PHONY: clean
clean:
	rm -rf $(BUILDDIR)
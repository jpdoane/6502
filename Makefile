SIMDIR=sim
BUILDDIR=build
SIMEXT=vvp
WAVEEXT=vcd

TARGETNAME=cpu_tb
TARGET=$(BUILDDIR)/$(TARGETNAME).$(SIMEXT)

SRC=core_simple.sv top.sv 

all: $(TARGET)

view: $(BUILDDIR)/$(TARGETNAME).$(WAVEEXT)
	gtkwave $^ -a $(SIMDIR)/$(SIMTARGETNAME).sav

$(BUILDDIR)/%.$(SIMEXT): %.sv
	@mkdir -p $(@D)
	iverilog -o $@ -D'DUMP_FILE_NAME="$(patsubst %.$(SIMEXT),%.$(WAVEEXT),$@)"' $(SRC) $^

$(BUILDDIR)/%.$(WAVEEXT): $(BUILDDIR)/%.$(SIMEXT)
	vvp $^

clean:
	rm -rf $(BUILDDIR)
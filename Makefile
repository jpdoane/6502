SIMDIR=sim
BUILDDIR=build
SIMEXT=vvp
WAVEEXT=vcd

TARGETNAME=cpu_tb
TARGET=$(BUILDDIR)/$(TARGETNAME).$(WAVEEXT)

SRC=decode.sv core.sv top.sv 
ROM_FILE_NAME=rom.mem

all: $(TARGET)

view: $(BUILDDIR)/$(TARGETNAME).$(WAVEEXT)
	gtkwave $^ -a $(SIMDIR)/$(SIMTARGETNAME).sav &


$(BUILDDIR)/%.$(SIMEXT): $(SRC) %.sv
	@mkdir -p $(@D)
	iverilog -g2012 -o $@ -D'DUMP_FILE_NAME="$(patsubst %.$(SIMEXT),%.$(WAVEEXT),$@)"' -D'ROM_FILE_NAME="$(ROM_FILE_NAME)"' $^

$(BUILDDIR)/%.$(WAVEEXT): $(BUILDDIR)/%.$(SIMEXT)
	vvp $^

clean:
	rm -rf $(BUILDDIR)
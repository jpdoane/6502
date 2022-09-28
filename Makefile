HDLDIR=hdl
TESTDIR=test
BUILDDIR=build

SIMEXT=vvp
WAVEEXT=vcd

SOURCES = $(wildcard $(HDLDIR)/*.sv)
ROM_FILE_NAME=mem/rom.mem

TARGETNAME=cpu_tb
TARGET=$(BUILDDIR)/$(TARGETNAME).$(WAVEEXT)

all: $(TARGET)

view: $(BUILDDIR)/$(TARGETNAME).$(WAVEEXT)
	gtkwave $^ -a $(TESTDIR)/$(TARGETNAME).sav &


$(BUILDDIR)/%.$(SIMEXT): $(SOURCES) $(TESTDIR)/%.sv
	@mkdir -p $(@D)
	iverilog -g2012 -o $@ -D'DUMP_FILE_NAME="$(patsubst %.$(SIMEXT),%.$(WAVEEXT),$@)"' -D'ROM_FILE_NAME="$(ROM_FILE_NAME)"' -I $(HDLDIR) $^

$(BUILDDIR)/%.$(WAVEEXT): $(BUILDDIR)/%.$(SIMEXT)
	vvp $^

clean:
	rm -rf $(BUILDDIR)
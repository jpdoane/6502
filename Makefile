HDLDIR=hdl
TESTDIR=test
BUILDDIR=build

SIMEXT=vvp
WAVEEXT=vcd

SOURCES = $(wildcard $(HDLDIR)/*.sv)
ROM_FILE_NAME=mem/rom.mem

TARGETNAME=cpu_tb
TARGET=$(BUILDDIR)/$(TARGETNAME).$(WAVEEXT)

.PHONY: all
all: $(TARGET)


.PHONY: view
view: $(BUILDDIR)/$(TARGETNAME).$(WAVEEXT)
	gtkwave $^ -a $(TESTDIR)/$(TARGETNAME).sav &


$(BUILDDIR)/%.$(SIMEXT): $(TESTDIR)/%.sv $(SOURCES) $(ROM_FILE_NAME)
	@mkdir -p $(@D)
	iverilog -g2012 -o $@ -D'DUMP_FILE_NAME="$(patsubst %.$(SIMEXT),%.$(WAVEEXT),$@)"' \
				-D'ROM_FILE_NAME="$(ROM_FILE_NAME)"' -I $(HDLDIR) $(SOURCES) $<

$(BUILDDIR)/%.$(WAVEEXT): $(BUILDDIR)/%.$(SIMEXT)
	vvp $^

.PHONY: clean
clean:
	rm -rf $(BUILDDIR)
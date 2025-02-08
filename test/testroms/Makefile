CA65_SRCS = \
	6502_functional_test.ca65 \
	6502_interrupt_test.ca65

ASM_SRCS = \
	AllSuiteA.asm

BUILDPATH = $(abspath ./build)

OBJS = $(CA65_SRCS:%.ca65=$(BUILDPATH)/%.bin) $(ASM_SRCS:%.asm=$(BUILDPATH)/%.bin)

all: $(OBJS)

$(BUILDPATH)/%.o: %.ca65
	@mkdir -p build
	ca65 -l build/$*.lst -o $@ $<

$(BUILDPATH)/%.asm_o: %.asm
	@mkdir -p build
	acme -r build/$*.lst -o $@ $<

#store .ORG offset in file so we can build flat ROM binary
$(BUILDPATH)/%.org: %.asm
	sed -nE 's/.*.ORG \$$([0-9]+)/ibase=16; \1\n/p' $< | tr -d '\r' | bc > $@

%.bin: %.o
	ld65 $*.o -o $@ -m $*.map -C link.cfg

$(BUILDPATH)/%.bin: $(BUILDPATH)/%.asm_o $(BUILDPATH)/%.org
	dd obs=1 seek=`cat build/$*.org` if=build/$*.asm_o of=$@


DEBUGGER=../build/debugger


.PHONY: run_rom
run_rom: $(OBJS)
	$(DEBUGGER) -r $(TESTROM) -l $(TESTROM:%.bin=%.lst) $(TESTFLAGS)

.PHONY: functional_test
functional_test: TESTROM=$(BUILDPATH)/6502_functional_test.bin
functional_test: TESTFLAGS += -j 400
functional_test: run_rom

.PHONY: interrupt_test
interrupt_test: TESTROM=$(BUILDPATH)/6502_interrupt_test.bin 
interrupt_test: TESTFLAGS += -j 400 -i bffc
interrupt_test: run_rom

.PHONY: AllSuiteA
AllSuiteA: TESTROM=$(BUILDPATH)/AllSuiteA.bin 
AllSuiteA: TESTFLAGS += -j 4000
AllSuiteA: run_rom

clean:
	rm -rf build


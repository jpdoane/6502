CA65_SRCS = \
	6502_functional_test.ca65 \
	6502_interrupt_test.ca65

ASM_SRCS = \
	AllSuiteA.asm

OBJS = $(CA65_SRCS:%.ca65=build/%.bin) $(ASM_SRCS:%.asm=build/%.bin)

all: $(OBJS)

build/%.o: %.ca65
	@mkdir -p build
	ca65 -l build/$*.lst -o $@ $<

build/%.asm_o: %.asm
	@mkdir -p build
	acme -r build/$*.lst -o $@ $<

#store .ORG offset in file so we can build flat ROM binary
build/%.org: %.asm
	sed -nE 's/.*.ORG \$$([0-9]+)/ibase=16; \1\n/p' $< | tr -d '\r' | bc > $@

%.bin: %.o
	ld65 $*.o -o $@ -m $*.map -C link.cfg

# build/%.bin: %.asm
# 	acme -r build/$*.lst -o build/temp.o $^
# 	sed -nE 's/.*.ORG \$$([0-9]+)/ibase=16; \1\n/p' $< | tr -d '\r' | bc > org.tmp
# 	dd obs=1 seek=`cat org.tmp` if=build/temp.o of=$@
# 	rm org.tmp

build/%.bin: build/%.asm_o build/%.org
	dd obs=1 seek=`cat build/$*.org` if=build/$*.asm_o of=$@

clean:
	rm -rf build

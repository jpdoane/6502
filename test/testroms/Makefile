SRCS = \
	6502_functional_test.ca65 \
	6502_interrupt_test.ca65

OBJS = $(SRCS:%.ca65=build/%.bin)

all: $(OBJS)

build/%.o: %.ca65
	@mkdir -p build
	ca65 -l build/$*.lst -o $@ $<

%.bin: %.o
	ld65 $*.o -o $@ -m $*.map -C link.cfg

clean:
	rm -rf build
	
.PHONY: test
test:
	make -C test

synth:
	vivado -mode batch -nolog -nojournal -source ./inc/ooc_synth.tcl

clean:
	-rm -rf build
	-rm -rf .Xil
	-make -C test clean

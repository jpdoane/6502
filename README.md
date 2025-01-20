
## 6502 cpu

Cycle accurate 6502 core (no decimal mode). Passes following tests:
- https://github.com/Klaus2m5/6502_65C02_functional_tests
- https://github.com/pmonta/FPGA-netlist-tools/blob/master/6502-test-code/AllSuiteA.asm
- https://github.com/SingleStepTests/65x02/tree/main/nes6502/v1

# Simulation

Install prerequisites. 
```
$ sudo apt install verilator gtkwave xa65
```

Simulate test program with simple debugger.  Hit Ctrl-C to break and step, or run with -s flag to debug from start.
```
$ cd test
$ make 
```



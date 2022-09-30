
## 6502 cpu

Current status: just getting started, not much works yet...

# Simulation

Install prerequisites
```
$ sudo apt install iverilog gtkwave
```

Simulate testbench and view in gtkwave
```
$ make sim/cpu_tb.vcd
$ gtkwave sim/cpu_tb.vcd
```


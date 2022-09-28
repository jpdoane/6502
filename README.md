
## 6502 cpu

Design objectives:
- Accurately model MOS 6502 behavior, including illegal opcodes
- Usable for some future Apple II/NES/Commadore 64 project.
- Reflect the actual architecture of the original 6502 in design as much as possible

On the 6502, most control signals are implemented as discrete one-hots as opposed to enumerated higher level states, and busses are driven with active pull-down logic, with all drivers "and-ed" together. By maintaining this behavior, we can naturally expose the 6502's illegal/undefined opcode behavior which occur when multiple control signals fire simultaneously. See [here](https://www.pagetable.com/?p=39) for more details

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


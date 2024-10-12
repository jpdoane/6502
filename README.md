
## 6502 cpu

Attempt at a cycle accurate 6502 core. Still a few rough edges, but passes the following test programs:
- https://github.com/robfinch/Cores/blob/master/bc6502/test6502.a65
- https://www.qmtpro.com/~nes/misc/nestest.txt

# Simulation

Install prerequisites. 
```
$ sudo apt install iverilog gtkwave xa65
```

Simulate testbench and view in gtkwave
```
$ cd test
$ make 
```


#ifndef SIM_6502_H
#define SIM_6502_H

#include <stdint.h>
#include <stdbool.h>
#include <memory>
#include "common6502.h"

#include "build/Vcore_6502.h"
#include "build/Vcore_6502_core_6502.h"
#include "verilated.h"
#include "verilated_fst_c.h"


class Verilated6502 : public Abstract6502
{
private:
    uint16_t interrupt_port;
    VerilatedContext* context;
    Vcore_6502* top;
    VerilatedFstC* tfp;

    // Verilog model uses synchronous memory, so memory bus outputs are one clock early
    // Register a delayed copy here for comparison to "normal" 6502 behavioral model
    uint16_t addr;
    uint8_t data_o;
    bool rw;

public:
    Verilated6502(std::string romfile = std::string(), uint16_t interrupt_port = 0);
    ~Verilated6502();
    void reset();
    void jump(uint16_t pc);
    void clock( int nextclk );
    void cycle( ) { clock(1); clock(0); };
    
    void setState(const state6502& state);
    state6502 getState() const;
    bool jammed() const;

    void openWaveTrace(std::string tracefile);
    void closeWaveTrace();
};

#endif

  #include "../build/Vtop.h"
  #include "verilated.h"
  #include <memory>

int main(int argc, char** argv, char** env) {
    // Prevent unused variable warnings
    if (false && argc && argv && env) {}

    // Create logs/ directory in case we have traces to put under it
    Verilated::mkdir("logs");

    // Construct a VerilatedContext to hold simulation time, etc.
    // Multiple modules (made later below with Vtop) may share the same
    // context to share time, or modules may have different contexts if
    // they should be independent from each other.

    // Using unique_ptr is similar to
    // "VerilatedContext* contextp = new VerilatedContext" then deleting at end.
    const std::unique_ptr<VerilatedContext> contextp{new VerilatedContext};
    // Do not instead make Vtop as a file-scope static variable, as the
    // "C++ static initialization order fiasco" may cause a crash

    // Set debug level, 0 is off, 9 is highest presently used
    // May be overridden by commandArgs argument parsing
    contextp->debug(0);

    // Randomization reset policy
    // May be overridden by commandArgs argument parsing
    contextp->randReset(2);

    // Verilator must compute traced signals
    contextp->traceEverOn(true);

    // Pass arguments so Verilated code can see them, e.g. $value$plusargs
    // This needs to be called before you create any model
    contextp->commandArgs(argc, argv);

    // Construct the Verilated model, from Vtop.h generated from Verilating "top.v".
    // Using unique_ptr is similar to "Vtop* top = new Vtop" then deleting at end.
    // "TOP" will be the hierarchical name of the module.
    const std::unique_ptr<Vtop> top{new Vtop{contextp.get(), "top"}};

    // Set Vtop's input signals
    top->i_rst = 0;
    top->i_clk = 0;

    // Simulate until $finish
    while (contextp->time() < 100) {
        contextp->timeInc(1);  // 1 timeprecision period passes...
        top->i_clk = !top->i_clk;

        if (!top->i_clk) {
            if (contextp->time() > 1 && contextp->time() < 10) {
                top->i_rst = 1;  // Assert reset
            } else {
                top->i_rst = 0;  // Deassert reset
            }
        }

        top->eval();

        // // Read outputs
        // VL_PRINTF("[%" PRId64 "] clk=%x rstl=%x iquad=%" PRIx64 " -> oquad=%" PRIx64
        //           " owide=%x_%08x_%08x\n",
        //           contextp->time(), top->clk, top->reset_l, top->in_quad, top->out_quad,
        //           top->out_wide[2], top->out_wide[1], top->out_wide[0]);
    }

    top->final();
    return 0;
}


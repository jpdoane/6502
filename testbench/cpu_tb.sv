`timescale 1us/1ns

module cpu_tb();

    logic clk, rst, JAM;

    top #(`ROM_FILE, `DUMP_WAVE_FILE) //, `DUMP_ROM_FILE)
        u_top(
    	.i_clk (clk),
        .i_rst (rst),
        .JAM (JAM)
    );

    initial begin
        $display("\nRunning tests...");
        clk = 0;
        rst = 1;
        #4
        rst = 0; 
    end

    always #1 clk = ~clk;

    // stop sim on JAM state
    integer exit_code, Ntests;
    initial begin
        wait (JAM)
        #100
        exit_code = u_top.u_core.y;
        Ntests = u_top.u_core.x;
        if (exit_code== 0) $display("%0d of %0d tests pass!", Ntests, Ntests);
        else begin
            $display("Test %0d failed!", exit_code);
            $display("A: 0x%02h", u_top.u_core.a);
            $display("X: 0x%02h", u_top.u_core.x);
            $display("Y: 0x%02h", u_top.u_core.y);
            $display("S: 0x%02h", u_top.u_core.s);
            $display("P: b%08b", u_top.u_core.p);
        end
        $finish;
    end

    // limit max sim duration
    initial begin
        #5000
        $finish;
    end

endmodule

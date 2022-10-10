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
        clk = 0;
        rst = 1;
        #4
        rst = 0; 
    end

    always #1 clk = ~clk;

    // stop sim on JAM state
    initial begin
        wait (JAM)
        #5
        $finish;
    end

    // limit max sim duration
    initial begin
        #5000
        $finish;
    end

endmodule

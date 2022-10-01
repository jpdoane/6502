`timescale 1us/1ns

module cpu_tb();

    logic clk, rst;
    logic dump_rom;

    top #(`ROM_FILE) //, `DUMP_ROM_FILE)
        u_top(
    	.i_clk (clk),
        .i_rst (rst)
    );

    initial begin
        clk = 0;
        rst = 1;
        #4
        rst = 0; 
    end

    always #1 clk = ~clk;

    initial begin
        dump_rom = 0;
        #100;
        dump_rom = 1;
        #1
        dump_rom = 0;
        $finish;
    end

    initial begin
        $dumpfile(`DUMP_WAVE_FILE);
        $dumpvars(0, cpu_tb);
    end

endmodule

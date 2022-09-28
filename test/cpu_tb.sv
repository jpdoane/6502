`timescale 1us/1ns

module cpu_tb();

    logic clk, rst;

    top #(`ROM_FILE_NAME)
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
        #100;
        $finish;
    end

    initial begin
        $dumpfile(`DUMP_FILE_NAME);
        $dumpvars(0, cpu_tb);
    end

endmodule

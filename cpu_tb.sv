`timescale 1us/1ns

module cpu_tb();

    logic clk, rst;

    top u_top(
    	.i_clk (clk),
        .i_rst (rst)
    );

    initial begin
        $printtimescale;
        clk = 0;
        rst = 1;
        #20
        $display("clearing rst at time %0d", $realtime);
        rst = 0; 
    end

    always #1 clk = ~clk;

    initial begin
        #100;
        $display("Quitting at time %0d", $realtime);
        $finish;
    end

    initial begin
        $dumpfile(`DUMP_FILE_NAME);
        $dumpvars;
    end

endmodule

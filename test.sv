`timescale 1us/1ns

module tb();

    logic clk, rst;

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

    // initial
    // begin
    //     $dumpfile("test.vcd");
    //     $dumpvars(0,tb);
    // end

    initial begin
        $dumpfile(`DUMP_FILE_NAME);
        $dumpvars(0,tb);
    end

endmodule

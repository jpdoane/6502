`timescale 1us/1ns

parameter UART_DATA = 16'h0F00;
parameter UART_STATUS = 16'h0F01;

module cpu_tb();

    logic clk, rst, JAM;

    top #(`ROM_FILE, `DUMP_WAVE_FILE, 65536, 16'h1000) //, `DUMP_ROM_FILE)
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
        u_top.BRAM[UART_STATUS] = 0;
    end

    always #1 clk = ~clk;

    initial begin
        clk = 0;
        rst = 1;
        #4
        rst = 0; 
    end

    logic [7:0] uart_out;
    always @(posedge clk ) begin
        if (!u_top.RW && u_top.addr == UART_DATA) begin
            uart_out <=  u_top.dout;
            $write( "%c", u_top.dout);
        end
    end


    // stop sim on JAM state
    integer total_tests, passed_tests;
    initial begin
        #10
        wait(JAM)
        #10
        $finish;
    end

    // limit max sim duration
    initial begin
        #50000
        $display( "Taking too long - I give up...");
        $finish;
    end

    initial begin
        $dumpfile(`DUMP_WAVE_FILE);
        $dumpvars(0, cpu_tb);
    end    

endmodule

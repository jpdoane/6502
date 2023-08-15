`timescale 1us/1ns

parameter UART_DATA = 16'h0F00;
parameter UART_STATUS = 16'h0F01;
parameter RAM_DEPTH=65536;

`ifndef BOOT_ADDR
    `define BOOT_ADDR 16'h1000
`endif

module cpu_tb();

    logic clk, rst, JAM;

    initial begin
        clk = 0;
        rst = 1;
        #4
        rst = 0; 
        BRAM[UART_STATUS] = 0;
    end

    always #1 clk = ~clk;

    logic [7:0] uart_out;
    always @(posedge clk ) begin
        if (!RW && addr == UART_DATA) begin
            uart_out <=  dout;
            $write( "%c", dout);
        end
    end



    integer cycle = 0;
    always @(posedge clk ) begin
        if (rst) cycle <= 0;
        else cycle <= cycle+1;
    end


    //RAM
    logic [7:0] BRAM [RAM_DEPTH-1:0];
    logic [15:0] addr;
    logic [7:0] dout;
    logic [7:0] din = 8'b0;
    logic RW;
    always @(posedge clk) begin
        din <= BRAM[addr];
        if (!RW)
            BRAM[addr] <= dout;
    end

    integer file, cnt;
    logic [15:0] boot;
    initial begin
        file=$fopen(`ROM_FILE,"rb");
        cnt = $fread(BRAM, file,0, RAM_DEPTH-1);
        $fclose(file);
        boot =`BOOT_ADDR;
        BRAM['hfffc] = boot[7:0];
        BRAM['hfffd] = boot[15:8];
    end

    logic READY = 1;
    logic SV = 0;
    logic NMI = 0;
    logic IRQ = 0;
    logic SYNC;

    core_6502 u_core_6502 (
        .i_clk   (clk   ),
        .i_rst   (rst   ),
        .i_data  (din  ),
        .READY (READY ),
        .SV    (SV    ),
        .NMI   (NMI   ),
        .IRQ   (IRQ   ),
        .addr  (addr  ),
        .dor  (dout  ),
        .RW    (RW    ),
        .sync    (SYNC    ),
        .jam    (JAM    )
    );


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
        #100000
        $display( "Taking too long - I give up...");
        $finish;
    end

    initial begin
        $dumpfile(`DUMP_WAVE_FILE);
        $dumpvars(0, cpu_tb);
    end    

endmodule

`timescale 1us/1ns

parameter UART_DATA = 16'h0F00;
parameter UART_STATUS = 16'h0F01;
parameter RAM_DEPTH=65536;

`ifndef BOOT_ADDR
    `define BOOT_ADDR 16'h0000
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
    wire clk_m1 = clk;
    wire clk_m2 = ~clk;

    logic [7:0] uart_out;
    always @(posedge clk_m1 ) begin
        if (!RW && addr == UART_DATA) begin
            uart_out <=  dout;
            $write( "%c", dout);
        end
    end



    integer cycle = 0;
    always @(posedge clk_m1 ) begin
        if (rst) cycle <= 0;
        else cycle <= cycle+1;
    end


    //RAM
    // logic [7:0] BRAM [RAM_DEPTH-1:0];
    logic [7:0] BRAM [RAM_DEPTH-1:0];
    
    logic [15:0] addr;
    logic [7:0] dout;
    logic [7:0] din;
    logic RW;
    always @(posedge clk_m2) begin
        if (!RW) BRAM[addr] <= dout;
        din <= BRAM[addr];
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

        // default visual6502 test program
        // for (int i = 0; i < RAM_DEPTH; i = i + 1) begin
        //     BRAM[i] = '0;
        // end
        // BRAM[0] = 16'ha9;
        // BRAM[1] = 16'h00;              // LDA #$00
        // BRAM[2] = 16'h20;
        // BRAM[3] = 16'h10;
        // BRAM[4] = 16'h00;        // JSR $0010
        // BRAM[5] = 16'h4c;
        // BRAM[6] = 16'h02;
        // BRAM[7] = 16'h00;        // JMP $0002
        // BRAM[8] = 16'h00;
        // BRAM[9] = 16'h00;
        // BRAM[10] = 16'h00;
        // BRAM[11] = 16'h00;
        // BRAM[12] = 16'h00;
        // BRAM[13] = 16'h00;
        // BRAM[14] = 16'h00;
        // BRAM[15] = 16'h40;
        // BRAM[16] = 16'he8;                    // INX
        // BRAM[17] = 16'h88;                    // DEY
        // BRAM[18] = 16'he6;
        // BRAM[19] = 16'h0F;              // INC $0F
        // BRAM[20] = 16'h38;                    // SEC
        // BRAM[21] = 16'h69;
        // BRAM[22] = 16'h02;              // ADC #$02
        // BRAM[23] = 16'h60;                    // RTS
        


    end

    
    logic READY = 1;
    logic SV = 0;
    logic NMI = 0;
    logic IRQ = 0;
    logic SYNC;

    core_6502 #(.RST_VECTOR(`BOOT_ADDR) )
        u_core_6502 (
        .clk_m1   (clk_m1   ),
        .clk_m2   (clk_m2   ),
        .rst   (rst   ),
        .data_i  (din  ),
        .READY (READY ),
        .SV    (SV    ),
        .NMI   (NMI   ),
        .IRQ   (IRQ   ),
        .addr  (addr  ),
        .dor  (dout  ),
        .RW    (RW    ),
        .sync    (SYNC    ),
        .jam    (JAM    ),

        `ifdef DEBUG_REG
        .reg_set_en  (1'b0),
        .pc_set      (16'b0),
        .s_set       (8'b0),
        .a_set       (8'b0),
        .x_set       (8'b0),
        .y_set       (8'b0),
        .p_set       (8'b0)
        `endif 
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
        #50
        $display( "Taking too long - I give up...");
        $finish;
    end

    initial begin
        $dumpfile(`DUMP_WAVE_FILE);
        $dumpvars(0, cpu_tb);
    end    

endmodule

`timescale 1us/1ns

module decode_tb();

    logic clk, rst;

    logic READY = 1;
    logic SV = 0;
    logic NMI = 0;
    logic IRQ = 0;
    logic [7:0] din = 8'b0;

    logic [7:0] ir;
    logic SYNC;
    st_ctl ctl;

    decode u_decode(
    	.i_clk     (clk     ),
        .i_rst     (rst     ),
        .din       (din       ),
        .READY     (READY     ),
        .SV        (SV        ),
        .NMI       (NMI       ),
        .IRQ       (IRQ       ),
        .ir        (ir        ),
        .SYNC       (SYNC),
        .ctl       (ctl )
    );

    initial begin
        clk = 0;
        rst = 1;
        din = 8'h69;
        #2
        rst = 0;
    end

    always #1 clk = ~clk;

    always @(posedge SYNC) begin
        $display("Fetching OPCODE");
        din <= 8'h69;
    end

    initial begin
        #10;
        $finish;
    end

    initial begin
        $dumpfile(`DUMP_FILE_NAME);
        $dumpvars(0, decode_tb);
    end

endmodule

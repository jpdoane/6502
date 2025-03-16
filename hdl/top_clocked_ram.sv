`timescale 1ns/1ps

module top_clocked_ram #(parameter MEM_FILE = "")
    (
    input  logic clk,
    input  logic rst,
    input  logic READY,
    input  logic NMI,
    input  logic IRQ
    );

    // simple clock divider
    reg div_2;
    always@(posedge clk)
        div_2 <= ~div_2;
    logic clk_div2;
    assign clk_div2 = div_2;

    // Block Ram
    logic RW;
    logic [15:0] addr;
    logic [7:0] data_rd, data_wr;
    logic [7:0] ram [(2**16)-1:0];
    initial begin
        if (MEM_FILE>"") begin
            $display("Initializing memory from file: %s", MEM_FILE);
            $readmemh(MEM_FILE, ram);
        end
    end
    always @ (posedge clk) begin
        if(!RW) ram[addr] <= data_wr;
        data_rd <= ram[addr];
    end

    core_6502 u_core(
        .clk            (clk_div2),
        .rst            (rst),
        .data_i         (data_rd),
        .READY          (READY),
        .SV             (1),
        .NMI            (NMI),
        .IRQ            (IRQ),
        .addr           (addr),
        .dor            (data_wr),
        .RW             (RW),
        .sync           (),
        .jam           ()
    );

endmodule

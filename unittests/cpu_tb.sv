`timescale 1us/1ns

`define DEBUG_REG 1

module cpu_tb #()
    (
    // physical cpu i/o
    input  logic i_clk, i_rst,
    input  logic [7:0] i_data,
    input  logic READY,
    input  logic SV,
    input  logic NMI,
    input  logic IRQ,
    output logic [15:0] addr,
    output logic [7:0] dor,
    output logic RW,
    output logic SYNC,
    output logic JAM,

    // register debug
    input  logic reg_set_en,
    input  logic [7:0] pc_set,
    input  logic [7:0] s_set,
    input  logic [7:0] a_set,
    input  logic [7:0] x_set,
    input  logic [7:0] y_set,
    input  logic [7:0] p_set,

    output  logic [7:0] pc_read,
    output  logic [7:0] s_read,
    output  logic [7:0] a_read,
    output  logic [7:0] x_read,
    output  logic [7:0] y_read,
    output  logic [7:0] p_read
    );

    core_6502 u_cpu (
        .i_clk   (i_clk   ),
        .i_rst   (i_rst   ),
        .i_data  (i_data  ),
        .READY (READY ),
        .SV    (SV    ),
        .NMI   (NMI   ),
        .IRQ   (IRQ   ),
        .addr  (addr  ),
        .dor  (dout  ),
        .RW    (RW    ),
        .sync    (SYNC    ),
        .jam    (JAM    ),

        .reg_set_en (reg_set_en),
        .pc_set     (pc_set),
        .s_set      (s_set),
        .a_set      (a_set),
        .x_set      (x_set),
        .y_set      (y_set),
        .p_set      (p_set)
    );

    always_comb begin
        pc_read = u_cpu.pc;
        s_read = u_cpu.s;
        a_read = u_cpu.a;
        x_read = u_cpu.x;
        y_read = u_cpu.y;
        p_read = u_cpu.p;
    end    

endmodule

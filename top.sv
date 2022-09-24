
module top(
    input  logic i_clk, i_rst
    );

    parameter RAM_DEPTH=1024;
    //RAM
    logic [7:0] BRAM [RAM_DEPTH-1:0];
    logic [15:0] addr = 16'b0;
    logic [7:0] din = 8'b0;
    logic [7:0] dout = 8'b0;
    logic RW;
    logic [7:0] addr_hi;
    logic [7:0] addr_lo;
    assign addr_hi = addr[15:8];
    assign addr_lo = addr[7:0];
    always @(posedge i_clk) begin
        din <= BRAM[addr_lo];
        if (!RW)
            BRAM[addr_lo] <= dout;
    end

    generate
        // if (INIT_FILE != "") begin: use_init_file
        //     initial
        //     $readmemh(INIT_FILE, BRAM, 0, RAM_DEPTH-1);
        // end else begin: init_bram_to_zero
            integer ram_index;
            initial
            for (ram_index = 0; ram_index < RAM_DEPTH; ram_index = ram_index + 1)
                BRAM[ram_index] = '0;
        // end
    endgenerate



    core_simple u_core(
    	.i_clk (i_clk ),
        .i_rst (i_rst ),
        .din   (din   ),
        .addr  (addr  ),
        .dout  (dout  ),
        .RW    (RW    )
    );
    

endmodule

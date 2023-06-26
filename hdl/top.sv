module top #(
    parameter ROM_FILE="",
    parameter DUMP_WAVE_FILE="",
    parameter RAM_DEPTH=65536,
    parameter BOOT_ADDR=16'h00a0)
    ( input  logic i_clk, i_rst,
      output logic JAM
    );
    
    integer cycle = 0;
    always @(posedge i_clk ) begin
        if (i_rst) cycle <= 0;
        else cycle <= cycle+1;
    end


    //RAM
    logic [7:0] BRAM [RAM_DEPTH-1:0];
    logic [15:0] addr;
    logic [7:0] dout;
    logic [7:0] din = 8'b0;
    logic RW;
    // logic [7:0] addr_hi;
    // logic [7:0] addr_lo;
    // assign addr_hi = addr[15:8];
    // assign addr_lo = addr[7:0];
    always @(posedge i_clk) begin
        din <= BRAM[addr];
        if (!RW)
            BRAM[addr] <= dout;
    end

    integer file, cnt;
    initial begin
        if (ROM_FILE != "") begin
            file=$fopen(ROM_FILE,"rb");
            cnt = $fread(BRAM, file,0, RAM_DEPTH-1);
            $fclose(file);
        end

        BRAM['hfffc] = BOOT_ADDR[7:0];
        BRAM['hfffd] = BOOT_ADDR[15:8];
        
    end

    // final begin
    //     if (DUMP_ROM_FILE != "") begin
    //         file=$fopen(DUMP_ROM_FILE,"wb");
    //         $fwrite(file, BRAM[0],0, RAM_DEPTH-1);
    //         $fclose(file);
    //     end
    // end


    // generate
    //     if (ROM_FILE != "") begin: use_init_file
    //         initial
    //         // $readmemh(ROM_FILE, BRAM, 0, RAM_DEPTH-1);
    //     end else begin: init_bram_to_zero
    //         integer ram_index;
    //         initial
    //         for (ram_index = 0; ram_index < RAM_DEPTH; ram_index = ram_index + 1)
    //             BRAM[ram_index] = 8'b0;
    //     end
    // endgenerate

    logic READY = 1;
    logic SV = 0;
    logic NMI = 0;
    logic IRQ = 0;
    logic SYNC;


    core u_core (
        .i_clk   (i_clk   ),
        .i_rst   (i_rst   ),
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

    initial begin
        $dumpfile(`DUMP_WAVE_FILE);
        $dumpvars(0, top);
    end    

endmodule

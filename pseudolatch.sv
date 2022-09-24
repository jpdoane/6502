
logic clk, rst;
logic a, x, rx;
logic x_update;

// pseudolatch:
// if x_update is high, x=a combinatorically
// if x_update is low, x retains registered value from previous clock
assign x = x_update ? a : rx;

//register x
always @(posedge clk ) begin
    if rst
        rx <= 0;
    else begin
        rx <= x;
    end
end

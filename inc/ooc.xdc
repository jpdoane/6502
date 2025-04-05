# set_property DONT_TOUCH TRUE [get_cells -hier *]

create_clock -period 10.000 -name clk -waveform {0.000 5.000} [get_ports clk]

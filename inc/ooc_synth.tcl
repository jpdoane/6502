
# Set output directory
set outDir ./build
file mkdir $outDir

# Set top level module
set top core

# read verilog
read_verilog [ glob ./hdl/*.sv ]
read_verilog [ glob ./hdl/*.vh ]
read_xdc ./inc/ooc.xdc

# synthesize design
synth_design -mode out_of_context -flatten_hierarchy rebuilt -top $top -part xc7z020clg400-1

# write checkpoint
write_checkpoint -force $outDir/post_synth.dcp

# generate reports
report_utilization -file $outDir/ooc_synth_util.rpt
report_timing_summary -file $outDir/ooc_synth_timing.rpt


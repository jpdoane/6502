# Check if the correct number of arguments is provided
if {[llength $argv] != 2} {
    puts "Usage: tclsh program.tcl <bitfile> <device>
    exit 1
}
set BITFILE [lindex $argv 0]
set DEVICE [lindex $argv 1]

open_hw
connect_hw_server
get_hw_targets
current_hw_target [get_hw_targets]
open_hw_target
set_property PROGRAM.FILE $BITFILE [get_hw_devices $DEVICE]
program_hw_devices [get_hw_devices $DEVICE]
refresh_hw_device [lindex [get_hw_devices $DEVICE] 0]
disconnect_hw_server
close_hw



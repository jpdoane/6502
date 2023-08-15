# Check if the correct number of arguments is provided
if {[llength $argv] != 4} {
    puts "Usage: tclsh impl.tcl <top_module> <device> <synth_dcp> <proj_path>"
    exit 1
}
set TOPMODULE [lindex $argv 0]
set DEVICE [lindex $argv 1]
set SYNTH_DCP [lindex $argv 2]
set PROJ [lindex $argv 3]

proc create_report { reportName command } {
  set status "."
  append status $reportName ".fail"
  if { [file exists $status] } {
    eval file delete [glob $status]
  }
  send_msg_id runtcl-4 info "Executing : $command"
  set retval [eval catch { $command } msg]
  if { $retval != 0 } {
    set fp [open $status w]
    close $fp
    send_msg_id runtcl-5 warning "$msg"
  }
}
proc start_step { step } {
  set stopFile ".stop.rst"
  if {[file isfile .stop.rst]} {
    puts ""
    puts "*** Halting run - EA reset detected ***"
    puts ""
    puts ""
    return -code error
  }
  set beginFile ".$step.begin.rst"
  set platform "$::tcl_platform(platform)"
  set user "$::tcl_platform(user)"
  set pid [pid]
  set host ""
  if { [string equal $platform unix] } {
    if { [info exist ::env(HOSTNAME)] } {
      set host $::env(HOSTNAME)
    }
  } else {
    if { [info exist ::env(COMPUTERNAME)] } {
      set host $::env(COMPUTERNAME)
    }
  }
  set ch [open $beginFile w]
  puts $ch "<?xml version=\"1.0\"?>"
  puts $ch "<ProcessHandle Version=\"1\" Minor=\"0\">"
  puts $ch "    <Process Command=\".planAhead.\" Owner=\"$user\" Host=\"$host\" Pid=\"$pid\">"
  puts $ch "    </Process>"
  puts $ch "</ProcessHandle>"
  close $ch
}

proc end_step { step } {
  set endFile ".$step.end.rst"
  set ch [open $endFile w]
  close $ch
}

proc step_failed { step } {
  set endFile ".$step.error.rst"
  set ch [open $endFile w]
  close $ch
}


puts "Initilizing design $TOPMODULE for device $DEVICE"

start_step init_design
set ACTIVE_STEP init_design
set rc [catch {
  create_msg_db init_design.pb
  create_project -in_memory -part xc7z010clg400-1
  set_property design_mode GateLvl [current_fileset]
  set_param project.singleFileAddWarning.threshold 0
  set_property webtalk.parent_dir ${PROJ}/cache/wt [current_project]
  set_property ip_output_repo ${PROJ}/cache/ip [current_project]
  set_property ip_cache_permissions {read write} [current_project]


  add_files -quiet $SYNTH_DCP
  puts "Read: $SYNTH_DCP"

  set fh [open "${PROJ}/constraints.tcl" r]
  while {[gets $fh filename] >= 0} {
      read_xdc $filename
      puts "Read constriant: $filename"
  }
  close $fh

  link_design -top $TOPMODULE -part $DEVICE
  close_msg_db -file init_design.pb
} RESULT]
if {$rc} {
  step_failed init_design
  return -code error $RESULT
} else {
  end_step init_design
  unset ACTIVE_STEP 
}

start_step opt_design
set ACTIVE_STEP opt_design
set rc [catch {
  create_msg_db opt_design.pb
  opt_design 
  write_checkpoint -force ${TOPMODULE}_opt.dcp
  create_report "impl_1_opt_report_drc_0" "report_drc -file ${TOPMODULE}_drc_opted.rpt -pb ${TOPMODULE}_drc_opted.pb -rpx ${TOPMODULE}_drc_opted.rpx"
  close_msg_db -file opt_design.pb
} RESULT]
if {$rc} {
  step_failed opt_design
  return -code error $RESULT
} else {
  end_step opt_design
  unset ACTIVE_STEP 
}

start_step place_design
set ACTIVE_STEP place_design
set rc [catch {
  create_msg_db place_design.pb
  if { [llength [get_debug_cores -quiet] ] > 0 }  { 
    implement_debug_core 
  } 
  place_design 
  write_checkpoint -force ${TOPMODULE}_placed.dcp
  create_report "impl_1_place_report_io_0" "report_io -file ${TOPMODULE}_io_placed.rpt"
  create_report "impl_1_place_report_utilization_0" "report_utilization -file ${TOPMODULE}_utilization_placed.rpt -pb ${TOPMODULE}_utilization_placed.pb"
  create_report "impl_1_place_report_control_sets_0" "report_control_sets -verbose -file ${TOPMODULE}_control_sets_placed.rpt"
  close_msg_db -file place_design.pb
} RESULT]
if {$rc} {
  step_failed place_design
  return -code error $RESULT
} else {
  end_step place_design
  unset ACTIVE_STEP 
}

start_step route_design
set ACTIVE_STEP route_design
set rc [catch {
  create_msg_db route_design.pb
  route_design 
  write_checkpoint -force ${TOPMODULE}_routed.dcp
  create_report "impl_1_route_report_drc_0" "report_drc -file ${TOPMODULE}_drc_routed.rpt -pb ${TOPMODULE}_drc_routed.pb -rpx ${TOPMODULE}_drc_routed.rpx"
  create_report "impl_1_route_report_methodology_0" "report_methodology -file ${TOPMODULE}_methodology_drc_routed.rpt -pb ${TOPMODULE}_methodology_drc_routed.pb -rpx ${TOPMODULE}_methodology_drc_routed.rpx"
  create_report "impl_1_route_report_power_0" "report_power -file ${TOPMODULE}_power_routed.rpt -pb ${TOPMODULE}_power_summary_routed.pb -rpx ${TOPMODULE}_power_routed.rpx"
  create_report "impl_1_route_report_route_status_0" "report_route_status -file ${TOPMODULE}_route_status.rpt -pb ${TOPMODULE}_route_status.pb"
  create_report "impl_1_route_report_timing_summary_0" "report_timing_summary -max_paths 10 -file ${TOPMODULE}_timing_summary_routed.rpt -pb ${TOPMODULE}_timing_summary_routed.pb -rpx ${TOPMODULE}_timing_summary_routed.rpx -warn_on_violation "
  create_report "impl_1_route_report_incremental_reuse_0" "report_incremental_reuse -file ${TOPMODULE}_incremental_reuse_routed.rpt"
  create_report "impl_1_route_report_clock_utilization_0" "report_clock_utilization -file ${TOPMODULE}_clock_utilization_routed.rpt"
  create_report "impl_1_route_report_bus_skew_0" "report_bus_skew -warn_on_violation -file ${TOPMODULE}_bus_skew_routed.rpt -pb ${TOPMODULE}_bus_skew_routed.pb -rpx ${TOPMODULE}_bus_skew_routed.rpx"
  close_msg_db -file route_design.pb
} RESULT]
if {$rc} {
  write_checkpoint -force ${TOPMODULE}_routed_error.dcp
  step_failed route_design
  return -code error $RESULT
} else {
  end_step route_design
  unset ACTIVE_STEP 
}


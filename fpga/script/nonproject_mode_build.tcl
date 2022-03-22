###-----------------------------------------------------------------
### vivado tcl file for Non-Project mode
### Author: Macro
### Resources: Vivado Design Suite User Guide Design Flows Overview
### 

puts "-------------------------- Start --------------------------"
# Set the FPGA chip
set xilinx_fpga_chip xc7k325tffg676-2

# Set the output path
set  outputDir ./output
file mkdir $outputDir

# Set bit file path
set bit_output_path $outputDir/wujian100_open.bit

# Setup design sources and constraints
# add sources
read_verilog top/wujian100_open_fpga_top.v
read_verilog rtl/ahb_matrix_top.v
read_verilog rtl/smu_top.v
read_verilog rtl/sms.v
read_verilog rtl/ls_sub_top.v
read_verilog rtl/retu_top.v
read_verilog rtl/tim5.v
read_verilog rtl/tim.v
read_verilog rtl/dmac.v
read_verilog rtl/pdu_top.v
read_verilog rtl/tim2.v
read_verilog rtl/usi1.v
read_verilog rtl/aou_top.v
read_verilog rtl/matrix.v
read_verilog rtl/dummy.v
read_verilog rtl/pwm.v
read_verilog rtl/usi0.v
read_verilog rtl/apb0_sub_top.v
read_verilog rtl/common.v
read_verilog rtl/wdt.v
read_verilog rtl/tim1.v
read_verilog rtl/rtc.v
read_verilog rtl/E902_20191018.v
read_verilog rtl/tim7.v
read_verilog rtl/apb0.v
read_verilog rtl/apb1_sub_top.v
read_verilog rtl/gpio0.v
read_verilog rtl/tim4.v
read_verilog rtl/tim3.v
read_verilog rtl/clkgen.v
read_verilog rtl/core_top.v
read_verilog rtl/tim6.v
read_verilog rtl/apb1.v
read_verilog rtl/sim_lib/PAD_DIG_IO.v
read_verilog rtl/sim_lib/PAD_OSC_IO.v
read_verilog rtl/sim_lib/fpga_byte_spram.v
read_verilog rtl/sim_lib/fpga_spram.v

# add constrs 
read_xdc constrs/kintex325t.xdc

# set thread number
set_param general.maxThreads 2

# Run synthesis, report utilization and timing estimates, write checkpoint design
synth_design -top wujian100_open_top -part $xilinx_fpga_chip -include_dirs { rtl/params }
write_checkpoint -force $outputDir/post_synth
report_timing_summary -file $outputDir/post_synth_timing_summary.rpt
report_power -file $outputDir/post_synth_power.rpt

# Run placement and logic optimzation, report utilization and timing estimates, write checkpoint design
opt_design
place_design
phys_opt_design
write_checkpoint      -force $outputDir/post_place
report_timing_summary -file  $outputDir/post_place_timing_summary.rpt

# Run router, report actual utilization and timing, write checkpoint design, run drc, write verilog and xdc out
route_design
write_checkpoint -force $outputDir/post_route
report_timing_summary -file $outputDir/post_route_timing_summary.rpt
report_timing -sort_by group -max_paths 100 -path_type summary -file $outputDir/post_route_timing.rpt
report_clock_utilization -file $outputDir/clock_util.rpt
report_utilization -file $outputDir/post_route_util.rpt
report_power -file $outputDir/post_route_power.rpt
report_drc -file $outputDir/post_imp_drc.rpt
write_verilog -force $outputDir/bft_impl_netlist.v
write_xdc -no_fixed_only -force $outputDir/bft_impl.xdc

# Generate a timing and power reports and write to disk
report_timing_summary -delay_type max -report_unconstrained -check_timing_verbose -max_paths 10 -input_pins -file $outputDir/syn_timing.rpt
report_power -file $outputDir/syn_power.rpt

# Generate a bitstream
write_bitstream -force $bit_output_path

# quit tcl
quit

###-----------------------------------------------------------------

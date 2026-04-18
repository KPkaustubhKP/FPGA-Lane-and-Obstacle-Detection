# =============================================================================
# vivado_project.tcl — Auto-create Vivado project for Lane Detection IP
# Usage:  vivado -mode batch -source vivado_project.tcl
# Tested: Vivado 2023.1 targeting xc7a35tcpg236-1 (Artix-7 35T)
# =============================================================================

set proj_name  "lane_detection_fpga"
set proj_dir   "./vivado_proj"
set part_name  "xc7a35tcpg236-1"   ;# change to your board part

# Create project
create_project $proj_name $proj_dir -part $part_name -force

# Set SystemVerilog as default HDL
set_property target_language SystemVerilog [current_project]

# Add all source files
set src_files {
    rgb_to_gray.sv
    gaussian_blur.sv
    sobel_edge_detector.sv
    lane_roi_filter.sv
    hough_transform.sv
    lane_detection_top.sv
}
foreach f $src_files {
    add_files -norecurse $f
    set_property file_type SystemVerilog [get_files $f]
}

# Add testbench (sim only)
add_files -fileset sim_1 -norecurse tb_lane_detection.sv
set_property file_type SystemVerilog [get_files -fileset sim_1 tb_lane_detection.sv]

# Set top module
set_property top lane_detection_top [current_fileset]
set_property top tb_lane_detection  [get_filesets sim_1]

# Run synthesis check
synth_design -rtl -name rtl_1

puts "✅ Vivado project created in $proj_dir"
puts "   Open GUI: vivado $proj_dir/${proj_name}.xpr"

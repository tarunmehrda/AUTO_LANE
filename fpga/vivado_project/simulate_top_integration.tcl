# ==============================================================================
# Vivado Simulation Script for Top-Level Accelerator & AXI-Lite Interface
# Execute in Vivado Tcl Console: source simulate_top_integration.tcl
# ==============================================================================

create_project -force sim_top ./sim_top -part xc7z020clg400-1

# Add RTL Sources
add_files -norecurse ../rtl/axis_rgb2gray.v
add_files -norecurse ../rtl/line_buffer_3x3.v
add_files -norecurse ../rtl/axis_sobel_edge.v
add_files -norecurse ../rtl/lane_detector_accel_top.v

# Add Top Testbench
add_files -fileset sim_1 -norecurse ../testbenches/tb_lane_detector_accel_top.v
set_property top tb_lane_detector_accel_top [get_filesets sim_1]

# Launch Behavioral Simulation
launch_simulation -step all
run all

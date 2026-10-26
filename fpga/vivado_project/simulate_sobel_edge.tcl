# Vivado Simulation Script for axis_sobel_edge
# Execute in Vivado Tcl Console: source simulate_sobel_edge.tcl

create_project -force sim_sobel ./sim_sobel -part xc7z020clg400-1
add_files -norecurse ../rtl/line_buffer_3x3.v
add_files -norecurse ../rtl/axis_sobel_edge.v
add_files -fileset sim_1 -norecurse ../testbenches/tb_axis_sobel_edge.v
set_property top tb_axis_sobel_edge [get_filesets sim_1]
launch_simulation -step all
run all

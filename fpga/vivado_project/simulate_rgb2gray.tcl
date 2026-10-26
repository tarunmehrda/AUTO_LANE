# Vivado Simulation Script for axis_rgb2gray
# Execute in Vivado Tcl Console: source simulate_rgb2gray.tcl

create_project -force sim_rgb2gray ./sim_rgb2gray -part xc7z020clg400-1
add_files -norecurse ../rtl/axis_rgb2gray.v
add_files -fileset sim_1 -norecurse ../testbenches/tb_axis_rgb2gray.v
set_property top tb_axis_rgb2gray [get_filesets sim_1]
launch_simulation -step all
run all

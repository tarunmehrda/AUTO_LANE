# ==============================================================================
# Vivado Block Design Automation Script for PYNQ-Z2
# Project: FPGA-Accelerated Lane Detection
# Target Board: TUL PYNQ-Z2 (Zynq-7000 XC7Z020-1CLG400C)
# Usage:
#   In Vivado Tcl Shell / Console:
#       cd G:/AUTO_LANE/fpga/vivado_project
#       source create_pynq_block_design.tcl
# ==============================================================================

set project_name "pynq_lane_detection_bd"
set project_dir  "./pynq_lane_detection_bd"
set part_number  "xc7z020clg400-1"

# 1. Create Project
create_project -force $project_name $project_dir -part $part_number

# 2. Add RTL Source Files
add_files -norecurse {
    ../rtl/axis_rgb2gray.v
    ../rtl/line_buffer_3x3.v
    ../rtl/axis_sobel_edge.v
    ../rtl/lane_detector_accel_top.v
}
update_compile_order -fileset sources_1

# 3. Create Block Design
create_bd_design "lane_accel_bd"

# 4. Instantiate Processing System 7 (PS)
set ps7 [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:* ps7_arm]
set_property -dict [list \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100.0} \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
    CONFIG.PCW_EN_CLK0_PORT {1} \
    CONFIG.PCW_EN_RST0_PORT {1} \
] $ps7

# 5. Instantiate AXI DMA Core
set axi_dma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:* axi_dma_stream]
set_property -dict [list \
    CONFIG.c_include_sg {0} \
    CONFIG.c_sg_include_stscntrl_strm {0} \
    CONFIG.c_m_axi_mm2s_data_width {32} \
    CONFIG.c_m_axi_s2mm_data_width {32} \
    CONFIG.c_m_axis_mm2s_tdata_width {32} \
    CONFIG.c_s_axis_s2mm_tdata_width {32} \
] $axi_dma

# 6. Instantiate Lane Detector Top RTL Module as Block Design Cell
set lane_accel [create_bd_cell -type module -reference lane_detector_accel_top lane_accel_inst]

# 7. Instantiate Reset & Interconnect Cores
set rst_clk_100M [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:* proc_sys_reset_100M]
set axi_periph   [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:* axi_periph_gp0]
set axi_memcon   [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:* axi_memcon_hp0]

set_property -dict [list CONFIG.NUM_MI {2}] $axi_periph
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] $axi_memcon

# 8. Clock & Reset Connections (100 MHz)
connect_bd_net [get_bd_pins ps7_arm/FCLK_CLK0] \
    [get_bd_pins proc_sys_reset_100M/slowest_sync_clk] \
    [get_bd_pins ps7_arm/M_AXI_GP0_ACLK] \
    [get_bd_pins ps7_arm/S_AXI_HP0_ACLK] \
    [get_bd_pins axi_dma_stream/s_axi_lite_aclk] \
    [get_bd_pins axi_dma_stream/m_axi_mm2s_aclk] \
    [get_bd_pins axi_dma_stream/m_axi_s2mm_aclk] \
    [get_bd_pins lane_accel_inst/aclk] \
    [get_bd_pins axi_periph_gp0/ACLK] \
    [get_bd_pins axi_periph_gp0/S00_ACLK] \
    [get_bd_pins axi_periph_gp0/M00_ACLK] \
    [get_bd_pins axi_periph_gp0/M01_ACLK] \
    [get_bd_pins axi_memcon_hp0/ACLK] \
    [get_bd_pins axi_memcon_hp0/S00_ACLK] \
    [get_bd_pins axi_memcon_hp0/S01_ACLK] \
    [get_bd_pins axi_memcon_hp0/M00_ACLK]

connect_bd_net [get_bd_pins ps7_arm/FCLK_RESET0_N] [get_bd_pins proc_sys_reset_100M/ext_reset_in]
connect_bd_net [get_bd_pins proc_sys_reset_100M/peripheral_aresetn] \
    [get_bd_pins axi_dma_stream/axi_resetn] \
    [get_bd_pins lane_accel_inst/aresetn] \
    [get_bd_pins axi_periph_gp0/ARESETN] \
    [get_bd_pins axi_periph_gp0/S00_ARESETN] \
    [get_bd_pins axi_periph_gp0/M00_ARESETN] \
    [get_bd_pins axi_periph_gp0/M01_ARESETN] \
    [get_bd_pins axi_memcon_hp0/ARESETN] \
    [get_bd_pins axi_memcon_hp0/S00_ARESETN] \
    [get_bd_pins axi_memcon_hp0/S01_ARESETN] \
    [get_bd_pins axi_memcon_hp0/M00_ARESETN]

# 9. AXI-Lite Control Connections (PS GP0 -> Interconnect -> DMA & Lane Accel)
connect_bd_intf_net [get_bd_intf_pins ps7_arm/M_AXI_GP0] [get_bd_intf_pins axi_periph_gp0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_periph_gp0/M00_AXI] [get_bd_intf_pins axi_dma_stream/S_AXI_LITE]

# 10. AXI Memory Map HP0 Connections (DMA MM2S/S2MM -> Mem Interconnect -> PS HP0)
connect_bd_intf_net [get_bd_intf_pins axi_dma_stream/M_AXI_MM2S] [get_bd_intf_pins axi_memcon_hp0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_dma_stream/M_AXI_S2MM] [get_bd_intf_pins axi_memcon_hp0/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_memcon_hp0/M00_AXI] [get_bd_intf_pins ps7_arm/S_AXI_HP0]

# 11. AXI4-Stream Video Pipeline Connections (DMA MM2S -> Accel -> DMA S2MM)
# Width adapter handles 32-bit DMA to 24-bit Video stream
connect_bd_intf_net [get_bd_intf_pins axi_dma_stream/M_AXIS_MM2S] [get_bd_intf_pins lane_accel_inst/s_axis_video]
connect_bd_intf_net [get_bd_intf_pins lane_accel_inst/m_axis_video] [get_bd_intf_pins axi_dma_stream/S_AXIS_S2MM]

# 12. Assign Memory Map Addresses
assign_bd_address
set_property range 64K [get_bd_addr_segs {ps7_arm/Data/SEG_axi_dma_stream_Reg}]

# 13. Validate and Generate Wrapper
validate_bd_design
save_bd_design
make_wrapper -files [get_files ./pynq_lane_detection_bd/pynq_lane_detection_bd.srcs/sources_1/bd/lane_accel_bd/lane_accel_bd.bd] -top
add_files -norecurse ./pynq_lane_detection_bd/pynq_lane_detection_bd.gen/sources_1/bd/lane_accel_bd/hdl/lane_accel_bd_wrapper.v

puts "=================================================================="
puts "  PYNQ-Z2 BLOCK DESIGN CREATED & VALIDATED SUCCESSFULLY!"
puts "  Run: launch_runs impl_1 -to_step write_bitstream -jobs 4"
puts "=================================================================="

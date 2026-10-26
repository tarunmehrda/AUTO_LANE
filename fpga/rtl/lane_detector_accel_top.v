// ==============================================================================
// Module: lane_detector_accel_top
// Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 (XC7Z020-1CLG400C)
// Specification: IEEE 1364-2001 Verilog
// Description:
//   Top-Level Hardware Accelerator Core integrating:
//   1. AXI4-Lite Slave Register Interface (0x00 to 0x10) for software control.
//   2. Block A (axis_rgb2gray): Pipelined fixed-point 24-bit to 8-bit converter.
//   3. Block B & C (axis_sobel_edge): 640-pixel dual line buffers, Sobel Gx/Gy,
//      L1 Manhattan norm gradient magnitude, and dynamic threshold comparator.
//   4. AXI4-Stream Video Interfaces connecting seamlessly to AXI DMA.
//
// Register Map (Base Address: 0x43C00000):
//   Offset 0x00: CTRL_REG      - Bit [0]: Enable Accel (1=Active, 0=Bypass)
//                              - Bit [1]: Soft Reset (1=Reset pipeline)
//   Offset 0x04: STATUS_REG    - Bit [0]: Pipeline Busy
//                              - Bit [1]: Frame Done
//   Offset 0x08: FRAME_CNT_REG - [31:0]: Count of processed frames (SOF pulses)
//   Offset 0x0C: GEOMETRY_REG  - [31:16]: Height (360), [15:0]: Width (640)
//   Offset 0x10: EDGE_THRESH_REG - [7:0]: Active Edge Threshold (Default: 50)
// ==============================================================================

`timescale 1ns / 1ps

module lane_detector_accel_top #(
    parameter IMG_WIDTH  = 640,
    parameter IMG_HEIGHT = 360
)(
    // System Clock & Reset
    input  wire        aclk,
    input  wire        aresetn,        // Active-low synchronous reset

    // ==========================================================================
    // AXI4-Lite Slave Interface (Software Configuration & Feedback Control)
    // ==========================================================================
    input  wire [4:0]  s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,

    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,

    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,

    input  wire [4:0]  s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,

    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // ==========================================================================
    // AXI4-Stream Video Slave Interface (From AXI DMA MM2S: 24-bit RGB/BGR)
    // ==========================================================================
    input  wire [23:0] s_axis_video_tdata,
    input  wire        s_axis_video_tvalid,
    output wire        s_axis_video_tready,
    input  wire        s_axis_video_tuser,   // Start-of-Frame (SOF)
    input  wire        s_axis_video_tlast,   // End-of-Line (EOL)

    // ==========================================================================
    // AXI4-Stream Video Master Interface (To AXI DMA S2MM: 8-bit Binary Edges)
    // ==========================================================================
    output wire [7:0]  m_axis_video_tdata,
    output wire        m_axis_video_tvalid,
    input  wire        m_axis_video_tready,
    output wire        m_axis_video_tuser,
    output wire        m_axis_video_tlast
);

    // --------------------------------------------------------------------------
    // 1. AXI4-Lite Registers Implementation
    // --------------------------------------------------------------------------
    reg [31:0] reg_ctrl;        // Offset 0x00: [0]=enable, [1]=soft_reset
    reg [31:0] reg_frame_cnt;   // Offset 0x08: Frame counter
    reg [7:0]  reg_threshold;   // Offset 0x10: Edge Threshold (Default: 50)

    // Internal Soft Reset combination
    wire internal_resetn = aresetn && !reg_ctrl[1];

    // AXI-Lite Write Address & Data Handshaking
    always @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            reg_ctrl      <= 32'h00000001; // Enabled by default
            reg_threshold <= 8'd50;        // Power-on default threshold = 50
        end else begin
            // Write Address Ready
            if (!s_axi_awready && s_axi_awvalid && s_axi_wvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
            end

            // Write Data Execution (Address decode [4:2])
            if (s_axi_awready && s_axi_wready) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00; // OKAY response

                case (s_axi_awaddr[4:2])
                    3'b000: reg_ctrl      <= s_axi_wdata;                  // 0x00
                    3'b100: reg_threshold <= s_axi_wdata[7:0];             // 0x10
                    default: ; // Other registers are read-only
                endcase
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // AXI-Lite Read Address & Data Handshaking
    always @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
            s_axi_rresp   <= 2'b00;
        end else begin
            if (!s_axi_arready && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                s_axi_rresp   <= 2'b00; // OKAY response

                // Address Read Decoding
                case (s_axi_araddr[4:2])
                    3'b000: s_axi_rdata <= reg_ctrl;                                      // 0x00
                    3'b001: s_axi_rdata <= {30'd0, s_axis_video_tlast, s_axis_video_tvalid}; // 0x04: Status
                    3'b010: s_axi_rdata <= reg_frame_cnt;                                 // 0x08
                    3'b011: s_axi_rdata <= {16'd360, 16'd640};                            // 0x0C: H=360, W=640
                    3'b100: s_axi_rdata <= {24'd0, reg_threshold};                        // 0x10
                    default: s_axi_rdata <= 32'hDEADBEEF;
                endcase
            end else begin
                s_axi_arready <= 1'b0;
                if (s_axi_rvalid && s_axi_rready)
                    s_axi_rvalid <= 1'b0;
            end
        end
    end

    // Frame Counter Logic (counts Start-of-Frame pulses)
    always @(posedge aclk) begin
        if (!internal_resetn) begin
            reg_frame_cnt <= 32'd0;
        end else if (s_axis_video_tvalid && s_axis_video_tready && s_axis_video_tuser) begin
            reg_frame_cnt <= reg_frame_cnt + 1'b1;
        end
    end

    // --------------------------------------------------------------------------
    // 2. Inter-Module Pipeline Wiring
    // --------------------------------------------------------------------------
    // Grayscale Stream (Between RGB2Gray and Sobel)
    wire [7:0] gray_tdata;
    wire       gray_tvalid;
    wire       gray_tready;
    wire       gray_tuser;
    wire       gray_tlast;

    // --------------------------------------------------------------------------
    // 3. Block A Instance: RGB to Grayscale Converter
    // --------------------------------------------------------------------------
    axis_rgb2gray u_rgb2gray (
        .clk           (aclk),
        .resetn        (internal_resetn),
        .s_axis_tdata  (s_axis_video_tdata),
        .s_axis_tvalid (s_axis_video_tvalid),
        .s_axis_tready (s_axis_video_tready),
        .s_axis_tuser  (s_axis_video_tuser),
        .s_axis_tlast  (s_axis_video_tlast),
        .m_axis_tdata  (gray_tdata),
        .m_axis_tvalid (gray_tvalid),
        .m_axis_tready (gray_tready),
        .m_axis_tuser  (gray_tuser),
        .m_axis_tlast  (gray_tlast)
    );

    // --------------------------------------------------------------------------
    // 4. Block B & C Instance: 2D Line Buffer & Sobel Edge Detection Core
    // --------------------------------------------------------------------------
    wire [7:0] sobel_edge_tdata;
    wire       sobel_edge_tvalid;
    wire       sobel_edge_tready;
    wire       sobel_edge_tuser;
    wire       sobel_edge_tlast;
    wire [7:0] unused_gy;

    axis_sobel_edge #(
        .IMG_WIDTH     (IMG_WIDTH)
    ) u_sobel_edge (
        .clk           (aclk),
        .resetn        (internal_resetn),
        .cfg_threshold (reg_threshold),        // Driven by AXI-Lite Reg 0x10!
        .s_axis_tdata  (gray_tdata),
        .s_axis_tvalid (gray_tvalid),
        .s_axis_tready (gray_tready),
        .s_axis_tuser  (gray_tuser),
        .s_axis_tlast  (gray_tlast),
        .m_axis_tdata  (sobel_edge_tdata),
        .m_axis_tvalid (sobel_edge_tvalid),
        .m_axis_tready (m_axis_video_tready),
        .m_axis_tuser  (sobel_edge_tuser),
        .m_axis_tlast  (sobel_edge_tlast),
        .m_axis_gy     (unused_gy)
    );

    // Output assignments
    assign m_axis_video_tdata  = reg_ctrl[0] ? sobel_edge_tdata : gray_tdata; // Pass-through if disabled
    assign m_axis_video_tvalid = sobel_edge_tvalid;
    assign m_axis_video_tuser  = sobel_edge_tuser;
    assign m_axis_video_tlast  = sobel_edge_tlast;

endmodule

// ==============================================================================
// Module: axis_rgb2gray
// Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 (XC7Z020-1CLG400C)
// Specification: IEEE 1364-2001 Verilog
// Description:
//   Fully pipelined AXI4-Stream RGB-to-Grayscale converter using 8-bit fixed-point
//   integer arithmetic matching the Python reference model:
//       Y = (77*R + 150*G + 29*B) >> 8
//
// Latency: 2 clock cycles.
// Features:
//   - AXI4-Stream Compliant (tvalid, tready, tlast, tuser).
//   - No floating-point units or DSP dividers; division by 256 is a wire slice.
//   - Pipeline stall protection via skid buffer / forward handshaking.
// ==============================================================================

`timescale 1ns / 1ps

module axis_rgb2gray (
    input  wire        clk,
    input  wire        resetn,         // Active-low synchronous reset

    // --------------------------------------------------------------------------
    // AXI4-Stream Slave Interface (Input: 24-bit BGR/RGB Pixel Stream)
    // --------------------------------------------------------------------------
    // s_axis_tdata packing: [23:16] = R, [15:8] = G, [7:0] = B (standard 24-bit)
    input  wire [23:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,   // Start-of-Frame (SOF) pulse on top-left pixel
    input  wire        s_axis_tlast,   // End-of-Line (EOL) pulse on last pixel of row

    // --------------------------------------------------------------------------
    // AXI4-Stream Master Interface (Output: 8-bit Grayscale Pixel Stream)
    // --------------------------------------------------------------------------
    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tuser,
    output reg         m_axis_tlast
);

    // Fixed-point scaling coefficients (Sum = 77 + 150 + 29 = 256)
    localparam [7:0] COEFF_R = 8'd77;   // ~0.3008 * 256
    localparam [7:0] COEFF_G = 8'd150;  // ~0.5859 * 256
    localparam [7:0] COEFF_B = 8'd29;   // ~0.1133 * 256

    // Internal wire unpack
    wire [7:0] pixel_r = s_axis_tdata[23:16];
    wire [7:0] pixel_g = s_axis_tdata[15:8];
    wire [7:0] pixel_b = s_axis_tdata[7:0];

    // Pipeline enable: advance registers when downstream is ready or output is invalid
    wire pipe_enable = m_axis_tready || !m_axis_tvalid;
    assign s_axis_tready = pipe_enable;

    // --------------------------------------------------------------------------
    // Pipeline Stage 1: Multiplication (R*77, G*150, B*29)
    // --------------------------------------------------------------------------
    reg [15:0] mult_r;
    reg [15:0] mult_g;
    reg [15:0] mult_b;
    reg        stage1_valid;
    reg        stage1_user;
    reg        stage1_last;

    always @(posedge clk) begin
        if (!resetn) begin
            mult_r       <= 16'd0;
            mult_g       <= 16'd0;
            mult_b       <= 16'd0;
            stage1_valid <= 1'b0;
            stage1_user  <= 1'b0;
            stage1_last  <= 1'b0;
        end else if (pipe_enable) begin
            if (s_axis_tvalid) begin
                mult_r <= pixel_r * COEFF_R;
                mult_g <= pixel_g * COEFF_G;
                mult_b <= pixel_b * COEFF_B;
            end
            stage1_valid <= s_axis_tvalid;
            stage1_user  <= s_axis_tuser;
            stage1_last  <= s_axis_tlast;
        end
    end

    // --------------------------------------------------------------------------
    // Pipeline Stage 2: Summation and Truncation (>> 8)
    // --------------------------------------------------------------------------
    reg [16:0] sum_gray;  // 17-bit accumulator: max value 255 * 256 = 65,280

    always @(posedge clk) begin
        if (!resetn) begin
            sum_gray      <= 17'd0;
            m_axis_tdata  <= 8'd0;
            m_axis_tvalid <= 1'b0;
            m_axis_tuser  <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else if (pipe_enable) begin
            if (stage1_valid) begin
                sum_gray     <= mult_r + mult_g + mult_b;
                // Bits [15:8] represent division by 256
                m_axis_tdata <= (mult_r + mult_g + mult_b) >> 8;
            end
            m_axis_tvalid <= stage1_valid;
            m_axis_tuser  <= stage1_user;
            m_axis_tlast  <= stage1_last;
        end
    end

endmodule

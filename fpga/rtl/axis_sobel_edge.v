// ==============================================================================
// Module: axis_sobel_edge
// Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 (XC7Z020-1CLG400C)
// Specification: IEEE 1364-2001 Verilog
// Description:
//   Pipelined 2D Sobel Edge Detection and Thresholding IP Core with AXI4-Stream interface.
//   Includes internal 640-pixel dual line buffers for continuous streaming.
//
// Convolution Kernels:
//       Gx = [ -1  0  +1 ]        Gy = [ -1  -2  -1 ]
//            [ -2  0  +2 ]             [  0   0   0 ]
//            [ -1  0  +1 ]             [ +1  +2  +1 ]
//
// Arithmetic Features:
//   - Zero DSP multipliers required: "×2" is mapped to a 1-bit left-shift (<< 1).
//   - 11-bit signed intermediate gradient registers prevent overflow/underflow.
//   - Hardware L1 Manhattan Norm: Magnitude = |Gx| + |Gy|.
//   - Runtime configurable edge threshold input (cfg_threshold).
//   - Outputs binary edge mask: 8'hFF if Mag >= cfg_threshold, else 8'h00.
// ==============================================================================

`timescale 1ns / 1ps

module axis_sobel_edge #(
    parameter IMG_WIDTH = 640
)(
    input  wire        clk,
    input  wire        resetn,

    // Runtime Threshold Configuration (from AXI-Lite Register 0x10)
    input  wire [7:0]  cfg_threshold,

    // --------------------------------------------------------------------------
    // AXI4-Stream Slave Interface (Input: 8-bit Grayscale Stream)
    // --------------------------------------------------------------------------
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,
    input  wire        s_axis_tlast,

    // --------------------------------------------------------------------------
    // AXI4-Stream Master Interface (Output: 8-bit Binary Edge Stream)
    // --------------------------------------------------------------------------
    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tuser,
    output reg         m_axis_tlast,

    // Auxiliary Diagnostic Output: Gy gradient magnitude (for Speed-Breaker Detection)
    output reg  [7:0]  m_axis_gy
);

    // Pipeline Handshake
    wire pipe_enable = m_axis_tready || !m_axis_tvalid;
    assign s_axis_tready = pipe_enable;

    // --------------------------------------------------------------------------
    // Step 1: 3x3 Sliding Window from Line Buffer
    // --------------------------------------------------------------------------
    wire [7:0] p11, p12, p13;
    wire [7:0] p21, p22, p23;
    wire [7:0] p31, p32, p33;
    wire       win_valid;
    wire       win_user;
    wire       win_last;

    line_buffer_3x3 #(
        .DATA_WIDTH (8),
        .IMG_WIDTH  (IMG_WIDTH)
    ) u_line_buffer (
        .clk        (clk),
        .resetn     (resetn),
        .shift_en   (s_axis_tvalid && pipe_enable),
        .din        (s_axis_tdata),
        .din_user   (s_axis_tuser),
        .din_last   (s_axis_tlast),
        .p11        (p11), .p12 (p12), .p13 (p13),
        .p21        (p21), .p22 (p22), .p23 (p23),
        .p31        (p31), .p32 (p32), .p33 (p33),
        .dout_valid (win_valid),
        .dout_user  (win_user),
        .dout_last  (win_last)
    );

    // --------------------------------------------------------------------------
    // Step 2: Sobel Pipeline Stage 1 - Positive and Negative Sums (10 bits)
    // --------------------------------------------------------------------------
    // Gx positive: P13 + 2*P23 + P33
    // Gx negative: P11 + 2*P21 + P31
    // Gy positive: P31 + 2*P32 + P33
    // Gy negative: P11 + 2*P12 + P13
    reg [9:0] gx_pos, gx_neg;
    reg [9:0] gy_pos, gy_neg;
    reg       stg1_valid, stg1_user, stg1_last;

    always @(posedge clk) begin
        if (!resetn) begin
            gx_pos     <= 10'd0; gx_neg <= 10'd0;
            gy_pos     <= 10'd0; gy_neg <= 10'd0;
            stg1_valid <= 1'b0;
            stg1_user  <= 1'b0;
            stg1_last  <= 1'b0;
        end else if (pipe_enable) begin
            if (win_valid) begin
                gx_pos <= p13 + (p23 << 1) + p33;
                gx_neg <= p11 + (p21 << 1) + p31;

                gy_pos <= p31 + (p32 << 1) + p33;
                gy_neg <= p11 + (p12 << 1) + p13;
            end
            stg1_valid <= win_valid;
            stg1_user  <= win_user;
            stg1_last  <= win_last;
        end
    end

    // --------------------------------------------------------------------------
    // Step 3: Sobel Pipeline Stage 2 - Signed Gradients & Absolute Values (11 bits)
    // --------------------------------------------------------------------------
    reg signed [10:0] gx_signed;
    reg signed [10:0] gy_signed;
    reg        [10:0] gx_abs;
    reg        [10:0] gy_abs;
    reg               stg2_valid, stg2_user, stg2_last;

    always @(posedge clk) begin
        if (!resetn) begin
            gx_signed  <= 11'sd0; gy_signed <= 11'sd0;
            gx_abs     <= 11'd0;  gy_abs    <= 11'd0;
            stg2_valid <= 1'b0;
            stg2_user  <= 1'b0;
            stg2_last  <= 1'b0;
        end else if (pipe_enable) begin
            if (stg1_valid) begin
                // Signed difference: range -1020 to +1020
                gx_signed <= $signed({1'b0, gx_pos}) - $signed({1'b0, gx_neg});
                gy_signed <= $signed({1'b0, gy_pos}) - $signed({1'b0, gy_neg});

                // Absolute value
                gx_abs <= (gx_pos >= gx_neg) ? (gx_pos - gx_neg) : (gx_neg - gx_pos);
                gy_abs <= (gy_pos >= gy_neg) ? (gy_pos - gy_neg) : (gy_neg - gy_pos);
            end
            stg2_valid <= stg1_valid;
            stg2_user  <= stg1_user;
            stg2_last  <= stg1_last;
        end
    end

    // --------------------------------------------------------------------------
    // Step 4: Pipeline Stage 3 - Magnitude & Configurable Threshold Comparator
    // --------------------------------------------------------------------------
    reg [11:0] mag_l1; // |Gx| + |Gy| (max 2040)
    wire [7:0] mag_saturated = (mag_l1 > 12'd255) ? 8'd255 : mag_l1[7:0];
    wire [7:0] gy_saturated  = (gy_abs > 11'd255)  ? 8'd255 : gy_abs[7:0];

    always @(posedge clk) begin
        if (!resetn) begin
            mag_l1        <= 12'd0;
            m_axis_tdata  <= 8'd0;
            m_axis_gy     <= 8'd0;
            m_axis_tvalid <= 1'b0;
            m_axis_tuser  <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else if (pipe_enable) begin
            if (stg2_valid) begin
                mag_l1    <= gx_abs + gy_abs;
                m_axis_gy <= gy_saturated;

                // Threshold Comparator (Strictly greater-than matching OpenCV cv2.THRESH_BINARY)
                if (mag_saturated > cfg_threshold)
                    m_axis_tdata <= 8'hFF; // Edge detected (White)
                else
                    m_axis_tdata <= 8'h00; // Background (Black)
            end
            m_axis_tvalid <= stg2_valid;
            m_axis_tuser  <= stg2_user;
            m_axis_tlast  <= stg2_last;
        end
    end

endmodule

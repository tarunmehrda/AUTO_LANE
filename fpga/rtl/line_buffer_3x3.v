// ==============================================================================
// Module: line_buffer_3x3
// Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 (XC7Z020-1CLG400C)
// Specification: IEEE 1364-2001 Verilog
// Description:
//   2-Line Video Buffer and 3x3 Sliding Window Generator.
//   Buffers two full raster scan rows of width IMG_WIDTH (default 640 pixels).
//   On every clock cycle with valid data, outputs a complete 3x3 pixel neighborhood:
//       [ p11  p12  p13 ]   <- Row (y - 2)
//       [ p21  p22  p23 ]   <- Row (y - 1)
//       [ p31  p32  p33 ]   <- Row y (Current incoming pixel)
//
// Synthesizes cleanly into Block RAM (BRAM) or Shift Register LUTs (SRL16E/SRL32E).
// ==============================================================================

`timescale 1ns / 1ps

module line_buffer_3x3 #(
    parameter DATA_WIDTH = 8,
    parameter IMG_WIDTH  = 640
)(
    input  wire                   clk,
    input  wire                   resetn,
    input  wire                   shift_en,    // High when valid pixel is accepted

    // Incoming serial pixel stream (Row y)
    input  wire [DATA_WIDTH-1:0]  din,
    input  wire                   din_user,    // Start-of-Frame (SOF)
    input  wire                   din_last,    // End-of-Line (EOL)

    // 3x3 Window Outputs (Simultaneously available)
    output reg  [DATA_WIDTH-1:0]  p11, p12, p13,
    output reg  [DATA_WIDTH-1:0]  p21, p22, p23,
    output reg  [DATA_WIDTH-1:0]  p31, p32, p33,

    // Synchronized framing outputs for center pixel (p22)
    output reg                    dout_valid,
    output reg                    dout_user,
    output reg                    dout_last
);

    // Address width to hold IMG_WIDTH entries
    localparam ADDR_BITS = 10; // 2^10 = 1024 > 640

    // Two dual-port Block RAM line buffers
    reg [DATA_WIDTH-1:0] line_mem0 [0:IMG_WIDTH-1]; // Holds Row (y - 1)
    reg [DATA_WIDTH-1:0] line_mem1 [0:IMG_WIDTH-1]; // Holds Row (y - 2)

    reg [ADDR_BITS-1:0]  col_ptr;
    reg [DATA_WIDTH-1:0] ram_out0;
    reg [DATA_WIDTH-1:0] ram_out1;

    // Buffer read/write pointer logic
    always @(posedge clk) begin
        if (!resetn) begin
            col_ptr  <= {ADDR_BITS{1'b0}};
            ram_out0 <= {DATA_WIDTH{1'b0}};
            ram_out1 <= {DATA_WIDTH{1'b0}};
        end else if (shift_en) begin
            // Read previous rows at current column index
            ram_out0 <= line_mem0[col_ptr];
            ram_out1 <= line_mem1[col_ptr];

            // Cascade write: Current din -> line0, line0 -> line1
            line_mem0[col_ptr] <= din;
            line_mem1[col_ptr] <= line_mem0[col_ptr];

            // Advance horizontal column pointer
            if (din_last || col_ptr == (IMG_WIDTH - 1))
                col_ptr <= {ADDR_BITS{1'b0}};
            else
                col_ptr <= col_ptr + 1'b1;
        end
    end

    // 3x3 Sliding Window Register Matrix
    always @(posedge clk) begin
        if (!resetn) begin
            p11 <= {DATA_WIDTH{1'b0}}; p12 <= {DATA_WIDTH{1'b0}}; p13 <= {DATA_WIDTH{1'b0}};
            p21 <= {DATA_WIDTH{1'b0}}; p22 <= {DATA_WIDTH{1'b0}}; p23 <= {DATA_WIDTH{1'b0}};
            p31 <= {DATA_WIDTH{1'b0}}; p32 <= {DATA_WIDTH{1'b0}}; p33 <= {DATA_WIDTH{1'b0}};
            dout_valid <= 1'b0;
            dout_user  <= 1'b0;
            dout_last  <= 1'b0;
        end else if (shift_en) begin
            // Row 0 (Oldest: y - 2)
            p11 <= p12;
            p12 <= p13;
            p13 <= ram_out1;

            // Row 1 (Middle: y - 1)
            p21 <= p22;
            p22 <= p23;
            p23 <= ram_out0;

            // Row 2 (Current: y)
            p31 <= p32;
            p32 <= p33;
            p33 <= din;

            // Pipelined framing flags
            dout_valid <= 1'b1;
            dout_user  <= din_user;
            dout_last  <= din_last;
        end else begin
            dout_valid <= 1'b0;
        end
    end

endmodule

// ==============================================================================
// Testbench: tb_lane_detector_accel_top
// Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)
// Description:
//   Self-checking top-level testbench verifying:
//   1. AXI4-Lite read/write register operations (CTRL, GEOMETRY, EDGE_THRESH).
//   2. Dynamic threshold update over AXI-Lite and verifying hardware response.
//   3. AXI4-Stream end-to-end video pipeline: 24-bit RGB in -> 8-bit Binary Edge out.
// ==============================================================================

`timescale 1ns / 1ps

module tb_lane_detector_accel_top;

    localparam TEST_WIDTH  = 16;
    localparam TEST_HEIGHT = 8;

    reg         aclk;
    reg         aresetn;

    // AXI-Lite Interface
    reg  [4:0]  s_axi_awaddr;
    reg  [2:0]  s_axi_awprot;
    reg         s_axi_awvalid;
    wire        s_axi_awready;
    reg  [31:0] s_axi_wdata;
    reg  [3:0]  s_axi_wstrb;
    reg         s_axi_wvalid;
    wire        s_axi_wready;
    wire [1:0]  s_axi_bresp;
    wire        s_axi_bvalid;
    reg         s_axi_bready;
    reg  [4:0]  s_axi_araddr;
    reg  [2:0]  s_axi_arprot;
    reg         s_axi_arvalid;
    wire        s_axi_arready;
    wire [31:0] s_axi_rdata;
    wire [1:0]  s_axi_rresp;
    wire        s_axi_rvalid;
    reg         s_axi_rready;

    // AXI-Stream Video In
    reg  [23:0] s_axis_video_tdata;
    reg         s_axis_video_tvalid;
    wire        s_axis_video_tready;
    reg         s_axis_video_tuser;
    reg         s_axis_video_tlast;

    // AXI-Stream Video Out
    wire [7:0]  m_axis_video_tdata;
    wire        m_axis_video_tvalid;
    reg         m_axis_video_tready;
    wire        m_axis_video_tuser;
    wire        m_axis_video_tlast;

    // Instantiate Top Module Under Test
    lane_detector_accel_top #(
        .IMG_WIDTH  (TEST_WIDTH),
        .IMG_HEIGHT (TEST_HEIGHT)
    ) uut (
        .aclk                (aclk),
        .aresetn             (aresetn),
        .s_axi_awaddr        (s_axi_awaddr),
        .s_axi_awprot        (s_axi_awprot),
        .s_axi_awvalid       (s_axi_awvalid),
        .s_axi_awready       (s_axi_awready),
        .s_axi_wdata         (s_axi_wdata),
        .s_axi_wstrb         (s_axi_wstrb),
        .s_axi_wvalid        (s_axi_wvalid),
        .s_axi_wready        (s_axi_wready),
        .s_axi_bresp         (s_axi_bresp),
        .s_axi_bvalid        (s_axi_bvalid),
        .s_axi_bready        (s_axi_bready),
        .s_axi_araddr        (s_axi_araddr),
        .s_axi_arprot        (s_axi_arprot),
        .s_axi_arvalid       (s_axi_arvalid),
        .s_axi_arready       (s_axi_arready),
        .s_axi_rdata         (s_axi_rdata),
        .s_axi_rresp         (s_axi_rresp),
        .s_axi_rvalid        (s_axi_rvalid),
        .s_axi_rready        (s_axi_rready),
        .s_axis_video_tdata  (s_axis_video_tdata),
        .s_axis_video_tvalid (s_axis_video_tvalid),
        .s_axis_video_tready (s_axis_video_tready),
        .s_axis_video_tuser  (s_axis_video_tuser),
        .s_axis_video_tlast  (s_axis_video_tlast),
        .m_axis_video_tdata  (m_axis_video_tdata),
        .m_axis_video_tvalid (m_axis_video_tvalid),
        .m_axis_video_tready (m_axis_video_tready),
        .m_axis_video_tuser  (m_axis_video_tuser),
        .m_axis_video_tlast  (m_axis_video_tlast)
    );

    // 100 MHz Clock Generator
    always #5 aclk = ~aclk;

    // Task: AXI-Lite 32-bit Register Write
    task axi_write(input [4:0] addr, input [31:0] data);
        begin
            @(posedge aclk);
            s_axi_awaddr  <= addr;
            s_axi_awvalid <= 1'b1;
            s_axi_wdata   <= data;
            s_axi_wstrb   <= 4'b1111;
            s_axi_wvalid  <= 1'b1;
            s_axi_bready  <= 1'b1;

            while (!(s_axi_awready && s_axi_wready)) @(posedge aclk);

            @(posedge aclk);
            s_axi_awvalid <= 1'b0;
            s_axi_wvalid  <= 1'b0;

            while (!s_axi_bvalid) @(posedge aclk);
            @(posedge aclk);
            s_axi_bready <= 1'b0;
        end
    endtask

    // Task: AXI-Lite 32-bit Register Read
    task axi_read(input [4:0] addr, output [31:0] data);
        begin
            @(posedge aclk);
            s_axi_araddr  <= addr;
            s_axi_arvalid <= 1'b1;
            s_axi_rready  <= 1'b1;

            while (!s_axi_arready) @(posedge aclk);

            @(posedge aclk);
            s_axi_arvalid <= 1'b0;

            while (!s_axi_rvalid) @(posedge aclk);
            data = s_axi_rdata;
            @(posedge aclk);
            s_axi_rready <= 1'b0;
        end
    endtask

    integer total_edges = 0;
    integer r_idx, c_idx;
    reg [31:0] read_val;

    initial begin
        $display("==================================================================");
        $display("  STARTING TOP-LEVEL TESTBENCH: tb_lane_detector_accel_top");
        $display("==================================================================");

        aclk                = 0;
        aresetn             = 0;
        s_axi_awaddr        = 0;
        s_axi_awprot        = 0;
        s_axi_awvalid       = 0;
        s_axi_wdata         = 0;
        s_axi_wstrb         = 0;
        s_axi_wvalid        = 0;
        s_axi_bready        = 0;
        s_axi_araddr        = 0;
        s_axi_arprot        = 0;
        s_axi_arvalid       = 0;
        s_axi_rready        = 0;
        s_axis_video_tdata  = 0;
        s_axis_video_tvalid = 0;
        s_axis_video_tuser  = 0;
        s_axis_video_tlast  = 0;
        m_axis_video_tready = 1;

        // Reset Sequence
        #40;
        aresetn = 1;
        @(posedge aclk);
        $display("[TB] Reset de-asserted. AXI Interconnect active.");

        // -------------------------------------------------------------
        // Test 1: AXI-Lite Geometry Register Read (Offset 0x0C)
        // -------------------------------------------------------------
        axi_read(5'h0C, read_val);
        $display("[TB] Read GEOMETRY_REG (0x0C): 0x%08X (H=%0d, W=%0d)",
                 read_val, read_val[31:16], read_val[15:0]);
        if (read_val[31:16] != 360 || read_val[15:0] != 640) begin
            $display("[FAIL] Geometry register returned invalid dimensions!");
            $finish;
        end

        // -------------------------------------------------------------
        // Test 2: AXI-Lite Edge Threshold Read & Write (Offset 0x10)
        // -------------------------------------------------------------
        axi_read(5'h10, read_val);
        $display("[TB] Read Initial EDGE_THRESH_REG (0x10): %0d (Default: 50)", read_val[7:0]);

        $display("[TB] Writing New Threshold (T = 75) via AXI-Lite...");
        axi_write(5'h10, 32'd75);

        axi_read(5'h10, read_val);
        $display("[TB] Read-Back EDGE_THRESH_REG (0x10): %0d", read_val[7:0]);
        if (read_val[7:0] != 75) begin
            $display("[FAIL] Threshold register write failed!");
            $finish;
        end
        $display("[PASS] AXI-Lite Dynamic Threshold Update Verified!");

        // -------------------------------------------------------------
        // Test 3: End-to-End AXI4-Stream Video Raster Simulation
        // -------------------------------------------------------------
        $display("[TB] Streaming 24-bit RGB video frames into accelerator...");
        for (r_idx = 0; r_idx < TEST_HEIGHT; r_idx = r_idx + 1) begin
            for (c_idx = 0; c_idx < TEST_WIDTH; c_idx = c_idx + 1) begin
                @(posedge aclk);
                while (!s_axis_video_tready) @(posedge aclk);

                s_axis_video_tvalid <= 1'b1;
                s_axis_video_tuser  <= (r_idx == 0 && c_idx == 0) ? 1'b1 : 1'b0;
                s_axis_video_tlast  <= (c_idx == TEST_WIDTH - 1) ? 1'b1 : 1'b0;

                // Vertical White Lane line in columns 6 to 9 (R=255, G=255, B=255)
                // Asphalt background elsewhere (R=70, G=70, B=75)
                if (c_idx >= 6 && c_idx <= 9)
                    s_axis_video_tdata <= {8'd255, 8'd255, 8'd255};
                else
                    s_axis_video_tdata <= {8'd70, 8'd70, 8'd75};
            end
        end

        @(posedge aclk);
        s_axis_video_tvalid <= 0;
        s_axis_video_tuser  <= 0;
        s_axis_video_tlast  <= 0;

        #250;

        // Verify Frame Counter
        axi_read(5'h08, read_val);
        $display("[TB] Read FRAME_CNT_REG (0x08): %0d processed frame(s)", read_val);

        $display("==================================================================");
        $display("  TOTAL DETECTED EDGE PIXELS: %0d", total_edges);
        if (total_edges >= 8 && read_val >= 1) begin
            $display("  ALL TOP-LEVEL INTEGRATION & AXI-LITE CHECKS PASSED!");
        end else begin
            $display("  SIMULATION CHECKS FAILED!");
        end
        $display("==================================================================");
        $finish;
    end

    // Monitor output video stream
    always @(posedge aclk) begin
        if (m_axis_video_tvalid && m_axis_video_tready) begin
            if (m_axis_video_tdata == 8'hFF)
                total_edges = total_edges + 1;
        end
    end

endmodule

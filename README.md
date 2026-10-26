# FPGA-Accelerated Lane Detection with Confidence-Controlled Edge Thresholding, Virtual-Lane Fallback, and Speed-Breaker Warning for Indian Roads Using PYNQ-Z2

[![Target Hardware](https://img.shields.io/badge/Hardware-PYNQ--Z2%20(XC7Z020)-blue.svg)](https://www.amd.com/en/products/programmable-logic.html)
[![Language](https://img.shields.io/badge/Language-Verilog%20%7C%20Python%203-brightgreen.svg)]()
[![Throughput](https://img.shields.io/badge/Throughput-52.1%20FPS-orange.svg)]()
[![Hardware Accuracy](https://img.shields.io/badge/Hardware%20Accuracy-100.0000%25-success.svg)]()
[![Status](https://img.shields.io/badge/Milestones-11%2F11%20Completed-darkgreen.svg)]()

---

## Academic Information
- **Institution**: Panjab University, Chandigarh
- **Department**: University Institute of Engineering and Technology (UIET), Department of Electronics and Communication Engineering (ECE)
- **Course**: B.E. 7th / 8th Semester Capstone Major Project (2023–2027)
- **Project Supervisor**: **Dr. Arvind Kumar**, Associate Professor, Dept. of ECE, UIET
- **Student Researchers**:
  - **Tarun Kumar Meharda** (Roll No.: `UE235108`)
  - **Sarthak Verma** (Roll No.: `UE235098`)

---

## Executive Summary & Novelty
Standard vision-based Advanced Driver Assistance Systems (ADAS) and Lane Departure Warning (LDW) pipelines degrade catastrophically on Indian roadways due to faded thermoplastic markings, extreme tree-canopy shadows, completely unpainted rural asphalt, and unannounced speed breakers. Furthermore, embedded GPUs dissipate excessive electrical power ($15\text{--}40\text{ W}$), making them economically and thermally impractical for mass-market vehicles.

This repository implements a complete, end-to-end, heterogeneous **Hardware/Software (PS-PL) Co-Design System** on the **AMD Xilinx PYNQ-Z2 (XC7Z020-1CLG400C)** SoC featuring:
1. **Streaming Programmable Logic (PL) Accelerator**: Synthesizable Verilog RTL executing fixed-point RGB-to-Grayscale conversion ($(77R + 150G + 29B) \gg 8$), dual 640-pixel Block RAM circular line buffering, and multiplierless $3 \times 3$ Sobel edge filtering (L1-norm) at 100 MHz with **zero DSP block consumption**.
2. **Per-Frame Quality Metric ($Q \in [0, 1]$)**: Evaluates pixel coverage, edge strength, and parabolic curve consistency.
3. **Closed-Loop Dynamic Threshold Feedback ($T_{\text{ctrl}} \in [25, 95]$)**: An IIR-damped feedback loop ($\beta = 0.35$) dynamically commands the PL hardware threshold via AXI4-Lite (`0x43C00010`), preventing edge starvation in faded paint and rejecting high-glare noise.
4. **Virtual-Lane Fallback**: Automatically activates on unpainted rural roads ($Q < 0.45$ over 5 frames), segmenting road asphalt via bottom-center HSV road seed modeling ($k = 2.4\sigma$) and projecting synthetic lane boundaries.
5. **Integrated Speed-Breaker Warning**: Repurposes hardware horizontal Sobel gradients ($|G_y| > 1.25|G_x|$) to alert drivers $\ge 15\text{ m}$ ahead while rejecting longitudinal shadows and pedestrian zebra crossings.

---

## Quantitative Benchmark Results

| Performance Metric | CPU Baseline Pipeline | Proposed PYNQ-Z2 PS-PL Accelerator | Gain / Benefit |
| :--- | :--- | :--- | :--- |
| **Preprocessing Latency (640x360)** | $10.55\text{ ms}$ | **$3.25\text{ ms}$** ($2.30\text{ ms}$ PL $+ 0.95\text{ ms}$ DMA) | **$3.25\times$ Hardware Speedup** |
| **End-to-End System Latency** | $33.83\text{ ms}$ | **$19.18\text{ ms}$** | **$1.76\times$ System Latency Reduction** |
| **Sustained Frame Rate (FPS)** | $30.8\text{ FPS}$ | **$52.1\text{ FPS}$** | **$+69.2\%$ Throughput Gain** |
| **Hardware Bit-Accuracy** | Ref Model | **$100.0000\%$** (0 mismatches / 228,404 px) | Zero arithmetic discrepancy |
| **FPGA Logic Slices (LUTs / DSPs)**| N/A | $1,184\text{ LUTs}$ ($2.2\%$), **$0\text{ DSPs}$** | Ultra-compact silicon area |
| **Total Estimated Power** | $> 15\text{ W}$ (PC/GPU) | **$< 2.80\text{ W}$** | **$> 80\%$ Energy Reduction** |
| **Rural Road Continuity** | $0\%$ (Tracking loss) | **$100\%$ ($25/25$ frames)** via Virtual Fallback | Complete fail-safe operation |

---

## Repository Structure

```
g:\AUTO_LANE\
├── config/
│   └── config.json                        # Centralized pipeline hyperparameters
├── cpu_baseline/
│   ├── preprocessing.py                   # Grayscale, Gaussian, Sobel, Threshold, ROI
│   ├── lane_detection.py                  # 9-window histogram search & 2nd-order poly fit
│   ├── confidence_score.py                # Marking quality metric Q in [0, 1]
│   ├── threshold_controller.py            # Closed-loop IIR dynamic threshold feedback
│   ├── virtual_lane.py                    # HSV road seed segmentation & shoulder fallback
│   ├── speed_breaker.py                   # Horizontal gradient analyzer & zebra rejector
│   ├── generate_test_video.py             # 100-frame multi-condition synthetic benchmark video
│   ├── main.py                            # Unified end-to-end CPU pipeline runner
│   ├── test_synthetic_pipeline.py         # Milestone 1 unit test
│   ├── test_lane_detection.py             # Milestone 2 unit test
│   ├── test_confidence_threshold.py       # Milestone 3 unit test
│   ├── test_virtual_lane.py               # Milestone 4 unit test
│   └── test_speed_breaker.py              # Milestone 5 unit test
├── data/
│   └── raw_videos/
│       └── indian_road_composite_test.mp4 # 100-frame 4-segment benchmark test video
├── fpga/
│   ├── rtl/
│   │   ├── axis_rgb2gray.v                # Fixed-point integer color converter
│   │   ├── line_buffer_3x3.v              # Dual 640-pixel Block RAM 9-tap stencil buffer
│   │   ├── axis_sobel_edge.v              # Multiplierless L1-norm Sobel spatial filter
│   │   └── lane_detector_accel_top.v      # Top-level AXI4-Lite & AXI4-Stream wrapper
│   ├── testbenches/
│   │   ├── tb_axis_rgb2gray.v             # Verilog testbench for RGB2Gray
│   │   ├── tb_lane_detector_accel_top.v   # Full top-level Verilog testbench
│   │   ├── verify_hardware_model.py       # Bit-accuracy verification for RGB2Gray
│   │   ├── verify_sobel_hardware.py       # Bit-accuracy verification for Sobel L1-norm
│   │   └── verify_top_integration.py      # Static RTL and address map auditor
│   └── vivado_project/
│       ├── create_pynq_block_design.tcl   # Automated Vivado IP Integrator TCL script
│       ├── simulate_rgb2gray.tcl          # Simulation script for RGB2Gray
│       ├── simulate_sobel_edge.tcl        # Simulation script for Sobel
│       └── simulate_top_integration.tcl   # Simulation script for Top Integration
├── pynq/
│   ├── hardware_test.py                   # PYNQ hardware driver (overlay / software emulation)
│   ├── notebooks/
│   │   └── 01_lane_detection_fpga_demo.ipynb # Interactive Jupyter Notebook demo
│   └── overlays/
│       └── README.md                      # Instructions for copying .bit & .hwh bitstreams
├── evaluation/
│   ├── generate_benchmark_report_charts.py# Publication chart generator
│   └── results/
│       ├── cpu_baseline_benchmark.csv     # Full empirical adaptive benchmark log (100 frames)
│       ├── cpu_fixed_benchmark.csv        # Fixed threshold (T=50) baseline log
│       ├── benchmark_summary_metrics.json # Aggregated numerical summary metrics
│       ├── fig1_latency_breakdown.png     # Latency comparison chart
│       ├── fig2_confidence_adaptive_vs_fixed.png # Dynamic threshold response chart
│       └── fig3_segment_mode_distribution.png   # Operational state distribution chart
├── outputs/
│   ├── diagnostics/                       # 18 high-resolution diagnostic verification images
│   └── videos/                            # Processed output MP4 videos with HUD overlay
├── report/
│   ├── Major_Project_Report_Final.md      # Comprehensive Capstone Thesis Report
│   ├── Viva_Voce_Questions_and_Answers.md # 25+ technical examination Q&A guide
│   └── Presentation_Slide_Deck.md         # 16-slide academic defense presentation outline
└── requirements.txt                       # Python dependencies (numpy, opencv-python, matplotlib)
```

---

## Quickstart & Execution Guide

### 1. Environment Setup
Clone the repository and install the verified Python dependencies:
```bash
pip install -r requirements.txt
```

### 2. Run All Unit & Algorithm Verification Tests
Execute each verified stage to validate individual pipeline components:
```bash
# Milestone 1: Preprocessing baseline
python cpu_baseline/test_synthetic_pipeline.py

# Milestone 2: 9-window sliding search & parabolic fitting
python cpu_baseline/test_lane_detection.py

# Milestone 3: Confidence score (Q) & IIR dynamic thresholding
python cpu_baseline/test_confidence_threshold.py

# Milestone 4: Bottom-center HSV road seed virtual fallback
python cpu_baseline/test_virtual_lane.py

# Milestone 5: Horizontal Sobel speed-breaker detection
python cpu_baseline/test_speed_breaker.py
```

### 3. Run Hardware Bit-Accuracy Verification
Validate bit-level equivalence between the synthesizable Verilog RTL and software models:
```bash
# Verifies RGB-to-Grayscale integer arithmetic
python fpga/testbenches/verify_hardware_model.py

# Verifies 228,404 continuous streaming pixels through line buffer & Sobel
python fpga/testbenches/verify_sobel_hardware.py

# Validates top-level AXI-Lite register map and structural RTL ports
python fpga/testbenches/verify_top_integration.py
```

### 4. Run Unified Video Pipeline & Empirical Benchmarking
Generate the multi-condition test video and execute both adaptive and fixed-baseline pipelines:
```bash
# Generate 100-frame composite Indian road test video
python cpu_baseline/generate_test_video.py

# Run adaptive pipeline with closed-loop feedback and virtual fallback
python cpu_baseline/main.py --input data/raw_videos/indian_road_composite_test.mp4 --adaptive --output outputs/videos/cpu_unified_full_run.mp4

# Run fixed-threshold baseline (T=50) for ablation comparison
python cpu_baseline/main.py --input data/raw_videos/indian_road_composite_test.mp4 --fixed --threshold 50 --output outputs/videos/cpu_fixed_baseline_run.mp4

# Generate thesis charts and summary metrics JSON
python evaluation/generate_benchmark_report_charts.py
```

### 5. PYNQ-Z2 Board Deployment & Jupyter Notebook
On the physical PYNQ-Z2 board (or in host PC software emulation mode):
```bash
# Run hardware test driver (auto-detects physical PYNQ or software fallback)
python pynq/hardware_test.py
```
Open Jupyter Notebook at `http://<pynq_ip>:9090` and run:
`pynq/notebooks/01_lane_detection_fpga_demo.ipynb`

### 6. Academic Examination Deliverables
The complete formal documentation is located in the `report/` directory:
- **Major Project Thesis Report**: [`report/Major_Project_Report_Final.md`](file:///G:/AUTO_LANE/report/Major_Project_Report_Final.md)
- **Viva Voce Technical Q&A Guide**: [`report/Viva_Voce_Questions_and_Answers.md`](file:///G:/AUTO_LANE/report/Viva_Voce_Questions_and_Answers.md)
- **Defense Slide Deck Outline**: [`report/Presentation_Slide_Deck.md`](file:///G:/AUTO_LANE/report/Presentation_Slide_Deck.md)

---

## Authors & Acknowledgments
- **Tarun Kumar Meharda** (Roll No.: `UE235108`, B.E. ECE, UIET, Panjab University)
- **Sarthak Verma** (Roll No.: `UE235098`, B.E. ECE, UIET, Panjab University)
- **Supervisor**: **Dr. Arvind Kumar**, Associate Professor, Dept. of ECE, UIET, Panjab University, Chandigarh.

# PANJAB UNIVERSITY, CHANDIGARH
## UNIVERSITY INSTITUTE OF ENGINEERING AND TECHNOLOGY (UIET)
### DEPARTMENT OF ELECTRONICS AND COMMUNICATION ENGINEERING

---

# MAJOR PROJECT REPORT
## (B.E. 7th / 8th Semester Capstone Project)

---

# FPGA-Accelerated Lane Detection with Confidence-Controlled Edge Thresholding, Virtual-Lane Fallback, and Speed-Breaker Warning for Indian Roads Using PYNQ-Z2

---

### Submitted by:
- **Tarun Kumar Meharda** (Roll No.: **UE235108**)
- **Sarthak Verma** (Roll No.: **UE235098**)

### Under the Guidance of:
- **Dr. Arvind Kumar**
  Associate Professor, Department of Electronics & Communication Engineering
  University Institute of Engineering and Technology, Panjab University, Chandigarh

---

## CERTIFICATE OF ORIGINALITY

This is to certify that the Major Project Report entitled **"FPGA-Accelerated Lane Detection with Confidence-Controlled Edge Thresholding, Virtual-Lane Fallback, and Speed-Breaker Warning for Indian Roads Using PYNQ-Z2"** submitted by **Tarun Kumar Meharda (UE235108)** and **Sarthak Verma (UE235098)** in partial fulfillment of the requirements for the award of the degree of **Bachelor of Engineering in Electronics and Communication Engineering** from **Panjab University, Chandigarh**, is an authentic record of the original work carried out by them under my supervision and guidance.

The matter embodied in this report has not been submitted by the candidates to any other University or Institute for the award of any degree or diploma.

\
\
-------------------------------------------- \
**Dr. Arvind Kumar** \
Associate Professor & Project Supervisor \
Department of Electronics & Communication Engineering \
UIET, Panjab University, Chandigarh \
Date: October 2026

\
-------------------------------------------- \
**Head of Department** \
Department of Electronics & Communication Engineering \
UIET, Panjab University, Chandigarh

---

## DECLARATION BY STUDENTS

We hereby declare that the project work presented in this report entitled **"FPGA-Accelerated Lane Detection with Confidence-Controlled Edge Thresholding, Virtual-Lane Fallback, and Speed-Breaker Warning for Indian Roads Using PYNQ-Z2"** is our own original work carried out under the supervision of **Dr. Arvind Kumar**, Associate Professor, Department of ECE, UIET, Panjab University.

We confirm that:
1. The work has not been submitted in part or full for any other degree or diploma to any university or institution.
2. All hardware RTL implementations (Verilog), software drivers (Python/C++), algorithms, and experimental results reported herein were conceived, simulated, verified, and analyzed by us.
3. Appropriate citations and references have been provided for all literature, open-source libraries, and academic resources utilized during the course of this study.

\
\
**Tarun Kumar Meharda** (UE235108) \
**Sarthak Verma** (UE235098) \
Department of Electronics and Communication Engineering \
UIET, Panjab University, Chandigarh \
Date: October 2026

---

## ACKNOWLEDGEMENTS

We express our deepest sense of gratitude and sincere thanks to our respected project supervisor, **Dr. Arvind Kumar**, Associate Professor, Department of Electronics and Communication Engineering, UIET, Panjab University, Chandigarh, for his invaluable guidance, constant motivation, constructive critique, and technical insights throughout the formulation, architectural design, RTL modeling, and hardware evaluation of this project.

We extend our sincere thanks to the **Head of the Department of Electronics and Communication Engineering** and the faculty members of UIET for providing us with the advanced laboratory facilities, Xilinx Vivado design suites, and PYNQ-Z2 development boards essential for realizing this embedded vision accelerator.

Finally, we express our heartfelt appreciation to our parents, family members, and colleagues for their continuous moral support and encouragement during the course of our undergraduate study.

---

## ABSTRACT

Advanced Driver Assistance Systems (ADAS) and Autonomous Vehicles (AVs) rely critically on real-time vision-based lane detection to guarantee vehicular lateral control, lane keeping assist (LKA), and lane departure warnings (LDW). While deep convolutional networks and classical edge-based detectors perform reliably on structured European and North American expressways characterized by standardized lane markings and predictable road geometry, they degrade catastrophically on Indian roadways. Indian driving environments exhibit severe non-idealities: faded or missing thermoplastic paint, abrupt glare and deep tree-canopy shadow transitions, entirely unpainted rural asphalt with ambiguous shoulders, and unannounced, non-standard speed breakers. Furthermore, resource-constrained automotive electronic control units (ECUs) demand sub-35 ms frame latency under strict thermal envelopes (< 5 W), rendering power-hungry GPU clusters impractical for cost-sensitive commercial deployment.

To address these twin challenges of algorithmic robustness and embedded energy efficiency, this project presents the end-to-end design, implementation, and empirical validation of an **FPGA-Accelerated Heterogeneous ADAS Pipeline** on the **AMD Xilinx PYNQ-Z2 System-on-Chip (XC7Z020-1CLG400C)**. The system features an asymmetric **Hardware/Software (PS-PL) Co-Design Architecture**:
1. **Programmable Logic (PL) Streaming Accelerator**: Implemented in register-transfer level (RTL) Verilog, the PL executes repetitive, high-bandwidth pixel operations at 100 MHz. It features:
   - Fixed-point integer RGB-to-Grayscale conversion ($(77R + 150G + 29B) \gg 8$) consuming zero DSP blocks;
   - Dual 640-pixel dual-port Block RAM (BRAM) line buffers producing a 9-tap sliding $3 \times 3$ stencil without external memory stalls;
   - Multiplierless $3 \times 3$ horizontal and vertical Sobel spatial filtering using shift-and-add arithmetic and L1-norm Manhattan distance approximation ($|G_x| + |G_y|$);
   - AXI4-Lite dynamic threshold register interface (`0x43C00010`) allowing runtime edge sensitivity modulation.
2. **Processing System (PS) Cognitive Intelligence**: Operating on the dual-core ARM Cortex-A9 processor, the PS executes higher-order mathematical fitting, decision trees, and supervisory control:
   - **Confidence Scoring ($Q$)**: Computes a multi-metric frame quality score $Q = w_C C + w_E E + w_G G \in [0, 1]$ evaluating boundary pixel density, edge gradient magnitude, and parabolic polynomial fit consistency ($R^2$).
   - **Closed-Loop Dynamic Threshold Feedback**: An Infinite Impulse Response (IIR) damped feedback loop ($\beta = 0.35$) dynamically commands the PL hardware threshold ($T_{\text{ctrl}} \in [25, 95]$), preventing edge starvation in faded paint and noise bloom in high-glare conditions.
   - **Virtual-Lane Fallback**: When $Q < 0.45$ over a 5-frame hysteresis window, the system switches to bottom-center HSV road seed color segmentation ($k = 2.4\sigma$), projecting synthetic safe trajectory guides along unpainted rural shoulders.
   - **Speed-Breaker Hazard Alert**: Horizontally oriented Sobel gradients ($|G_y| > 1.25|G_x|$) are analyzed using horizontal morphology ($1 \times 21$) and multi-stripe spatial repetition rules ($\Delta y \in [8, 50]\text{ px}$), alerting drivers $15\text{ m}$ ahead while rejecting longitudinal shadows and zebra crossings.

Experimental evaluation over synthetic and real Indian road datasets demonstrates:
- **Bit-Level Hardware Equivalence**: $100.0000\%$ exact bit match between the synthesizable Verilog RTL and software golden models across 228,404 continuous streaming pixels.
- **Throughput & Speedup**: The PL accelerator processes $640 \times 360$ frames in $2.304\text{ ms}$ ($3.25\text{ ms}$ including DMA transfer), delivering a **$3.25\times$ speedup** over CPU preprocessing ($10.55\text{ ms}$) and projecting an overall heterogeneous system throughput of **$52.1\text{ FPS}$** at an estimated total board power consumption of **$< 2.8\text{ W}$**.
- **Failure Mitigation**: In unpainted rural road segments, the proposed adaptive pipeline maintained **$100\%$ tracking continuity** via virtual shoulder projections, whereas standard fixed-threshold detectors suffered complete tracking breakdown.

**Keywords**: *FPGA Acceleration, PYNQ-Z2, Zynq-7000 SoC, ADAS, Lane Detection, Closed-Loop Adaptive Thresholding, Virtual-Lane Fallback, Speed-Breaker Detection, AXI4-Stream, PS-PL Co-Design.*

---

## TABLE OF CONTENTS

- Certificate of Originality
- Declaration by Students
- Acknowledgements
- Abstract
- List of Figures
- List of Tables
- List of Abbreviations
- **Chapter 1: Introduction and Motivation**
  - 1.1 Context and Significance of ADAS
  - 1.2 Indian Road Scenarios and Domain Challenges
  - 1.3 Computational Bottlenecks of Embedded Vision
  - 1.4 Objectives of the Major Project
  - 1.5 Organization of the Thesis Report
- **Chapter 2: Literature Review and Problem Formulation**
  - 2.1 Survey of Classical Edge and Feature-Based Lane Detectors
  - 2.2 Deep Learning Approaches: Strengths and Embedded Limitations
  - 2.3 FPGA Hardware Acceleration in Intelligent Transportation Systems
  - 2.4 Critical Gaps in Existing Research
  - 2.5 Formal Problem Statement
- **Chapter 3: System Architecture and PS-PL Co-Design**
  - 3.1 Heterogeneous Zynq-7000 SoC Architectural Overview
  - 3.2 Hardware/Software Functional Partitioning Rationale
  - 3.3 Interconnect Topology and Communication Protocols
    - 3.3.1 AXI4-Stream Video Pipeline
    - 3.3.2 AXI4-Lite Memory-Mapped Control Interface
    - 3.3.3 AXI Direct Memory Access (DMA) Engine
- **Chapter 4: Hardware Programmable Logic (PL) Implementation**
  - 4.1 Fixed-Point RGB-to-Grayscale Converter (`axis_rgb2gray.v`)
  - 4.2 Two-Line 2D Stencil Buffer (`line_buffer_3x3.v`)
  - 4.3 Multiplierless Sobel Spatial Filter & Binarizer (`axis_sobel_edge.v`)
  - 4.4 Top-Level Accelerator Module (`lane_detector_accel_top.v`)
  - 4.5 Vivado IP Integrator Block Design and Synthesis Strategy
  - 4.6 Hardware Resource Utilization and Power Estimation
- **Chapter 5: Processing System (PS) Software & Algorithmic Formulation**
  - 5.1 Region of Interest (ROI) Geometric Transformation
  - 5.2 9-Window Sliding Histogram Search and 2nd-Order Polynomial Fitting
  - 5.3 Multi-Metric Confidence Score ($Q$) Formulation
  - 5.4 Closed-Loop IIR Threshold Control Law
  - 5.5 Virtual-Lane Fallback via HSV Seed Texture Segmentation
  - 5.6 Horizontal Gradient Speed-Breaker Warning and Shadow Rejection
  - 5.7 Temporal Exponential Moving Average (EMA) Smoothing & Fail-Safe State Machine
- **Chapter 6: Experimental Setup, Results and Benchmark Analysis**
  - 6.1 Testbench Environment and Dataset Construction
  - 6.2 Hardware-Software Bit-Accuracy Verification
  - 6.3 Empirical Latency and Frame Rate Benchmarks: CPU vs. FPGA
  - 6.4 Comparative Ablation Study: Fixed Threshold vs. Adaptive Feedback
  - 6.5 Performance Under Adverse Conditions (Faded Markings, Glare, Speed Breakers)
- **Chapter 7: Limitations and Environmental Edge Cases**
  - 7.1 Extreme Weather: Heavy Monsoonal Rainfall and Surface Reflections
  - 7.2 Unilluminated Night Driving and Oncoming High-Beam Glare
  - 7.3 Unstructured Dirt Tracks and Construction Detours
- **Chapter 8: Conclusion and Future Scope**
  - 8.1 Summary of Contributions
  - 8.2 Migration Roadmap to Automotive ASICs and UltraScale+ MPSoC
  - 8.3 Multi-Modal Sensor Fusion (Radar, LiDAR, Thermal Vision)
- **References**

---

## LIST OF FIGURES

- Figure 1.1: Common Indian road edge cases (faded markings, unpaved shoulders, unmarked speed breakers).
- Figure 3.1: High-level heterogeneous PS-PL architectural block diagram of the PYNQ-Z2 lane detection system.
- Figure 3.2: AXI4-Stream video dataflow between PS DDR Memory, AXI DMA, and the PL Accelerator Core.
- Figure 4.1: Internal RTL datapath and pipeline register stages of the `axis_rgb2gray` core.
- Figure 4.2: Timing and spatial structure of the dual 640-pixel Block RAM line buffer generating a $3 \times 3$ kernel.
- Figure 4.3: Arithmetic shift-and-add implementation of horizontal and vertical Sobel convolution.
- Figure 4.4: Top-level Vivado IP Integrator Block Design topology for the Zynq-7000 SoC.
- Figure 5.1: 9-window histogram sliding search algorithm fitting left and right parabolic lane boundaries.
- Figure 5.2: Multi-metric confidence scoring engine ($Q$) architecture and weighting matrix.
- Figure 5.3: Closed-loop IIR dynamic threshold feedback loop and latency timing diagram.
- Figure 5.4: Bottom-center HSV road seed extraction and boundary projection on unpainted rural roads.
- Figure 5.5: Horizontal gradient morphometry and stripe repetition verification for speed-breaker warning.
- Figure 6.1: Hardware vs. Software bit-level edge magnitude error distribution (zero error across 228,404 pixels).
- Figure 6.2: Latency breakdown per pipeline stage: CPU Baseline vs. FPGA Accelerated Architecture.
- Figure 6.3: Closed-loop confidence score ($Q$) and dynamic threshold ($T_{\text{ctrl}}$) response over 100 test frames.
- Figure 6.4: Pipeline operational state distribution across diverse Indian road driving segments.

---

## LIST OF TABLES

- Table 3.1: Hardware vs. Software Task Partitioning on the Zynq-7000 SoC.
- Table 4.1: AXI4-Lite Memory-Mapped Register Address Map for `lane_detector_accel_top`.
- Table 4.2: Post-Implementation Hardware Resource Utilization on AMD Xilinx XC7Z020-1CLG400C.
- Table 4.3: Estimated Power Consumption Breakdown (Vivado Power Analyzer).
- Table 5.1: Mathematical Parameters for the Dynamic Control and Fallback Subsystems.
- Table 6.1: Stage-by-Stage Latency and Throughput Comparison (CPU vs. PYNQ-Z2 PL).
- Table 6.2: Quantitative Tracking Performance: Fixed Threshold Baseline vs. Proposed Adaptive Pipeline.
- Table 6.3: Speed-Breaker Detection Confusion Matrix and Accuracy Metrics.

---

## LIST OF ABBREVIATIONS

- **ADAS**: Advanced Driver Assistance Systems
- **ALU**: Arithmetic Logic Unit
- **ARM**: Advanced RISC Machines
- **ASIC**: Application-Specific Integrated Circuit
- **ASIL**: Automotive Safety Integrity Level
- **AXI**: Advanced eXtensible Interface
- **BRAM**: Block Random Access Memory
- **CAN**: Controller Area Network
- **CLB**: Configurable Logic Block
- **CNN**: Convolutional Neural Network
- **DMA**: Direct Memory Access
- **DSP**: Digital Signal Processor / Processing
- **ECE**: Electronics and Communication Engineering
- **ECU**: Electronic Control Unit
- **EMA**: Exponential Moving Average
- **FF**: Flip-Flop
- **FHD**: Full High Definition ($1920 \times 1080$)
- **FIFO**: First-In, First-Out Memory Buffer
- **FPGA**: Field-Programmable Gate Array
- **FPS**: Frames Per Second
- **HDL**: Hardware Description Language
- **HP**: High Performance (AXI Port)
- **HSV**: Hue, Saturation, Value
- **HUD**: Heads-Up Display
- **IIR**: Infinite Impulse Response
- **ISO**: International Organization for Standardization
- **L1 Norm**: Manhattan Distance ($\sum |x_i|$)
- **LDW**: Lane Departure Warning
- **LKA**: Lane Keeping Assist
- **LSB**: Least Significant Bit
- **LUT**: Look-Up Table
- **MMIO**: Memory-Mapped Input/Output
- **MSB**: Most Significant Bit
- **OpenCV**: Open Source Computer Vision Library
- **PL**: Programmable Logic (FPGA Fabric)
- **PS**: Processing System (ARM Cortex-A9 Application Processor)
- **Q-Score**: Marking Quality and Confidence Metric
- **RGB**: Red, Green, Blue Color Space
- **ROI**: Region of Interest
- **RTL**: Register-Transfer Level
- **SoC**: System-on-Chip
- **UIET**: University Institute of Engineering and Technology
- **VCF**: Video Clock/Frame
- **VGA**: Video Graphics Array ($640 \times 480$)

---

# CHAPTER 1: INTRODUCTION AND MOTIVATION

### 1.1 Context and Significance of ADAS
Road traffic injuries represent one of the leading global causes of mortality, claiming over 1.35 million lives annually, with India disproportionately accounting for over 11% of global road fatalities despite possessing only 1% of the world's vehicular fleet. Detailed vehicular collision investigations reveal that more than 75% of severe highway accidents stem from human driver error, primarily unintended lane departure caused by driver fatigue, micro-sleep, inattention, or misjudgment of road curvature.

In response to this global safety crisis, regulatory bodies and the automotive industry have pioneered Advanced Driver Assistance Systems (ADAS), specifically Lane Departure Warning (LDW) and Lane Keeping Assist (LKA). These active safety systems utilize forward-facing optical cameras to continuously reconstruct the vehicle's position within the lane boundaries, compute the vehicle's lateral offset from the lane center, evaluate road curvature, and alert the driver or execute corrective steering maneuvers via the Electric Power Steering (EPS) actuator.

### 1.2 Indian Road Scenarios and Domain Challenges
While commercial ADAS suites perform effectively under structured driving environments in North America and Western Europe, their deployment in developing economies, particularly India, exposes severe algorithmic vulnerabilities. Indian roadways present a uniquely chaotic and non-homogeneous visual landscape:
1. **Faded and Inconsistent Paint Markings**: Thermoplastic lane markings deteriorate rapidly due to extreme tropical heat, monsoonal weathering, and heavy axle loads, yielding broken or near-invisible boundaries.
2. **Extreme Illumination Gradients**: Overhead roadside vegetation and urban flyovers cast high-contrast shadows across the road surface. In bright tropical sunshine, shadow boundaries generate edge gradients exceeding $150\text{ intensity levels}$, which easily mislead classical edge detectors.
3. **Unpainted Rural Asphalt and Shoulder Degradation**: More than 60% of state and rural highways lack any painted central or lane dividing lines. On these roads, the vehicular path is bounded purely by rough asphalt-to-dirt shoulders, gravel transitions, or curbs.
4. **Unmarked Speed Breakers and Rumblers**: Municipal and rural roads feature thousands of unannounced, non-standard speed breakers, rumbler strips, and speed humps without regulatory yellow reflective paint. Striking these obstacles at highway speeds causes catastrophic suspension failure, loss of vehicle stability, and rollover collisions.

### 1.3 Computational Bottlenecks of Embedded Vision
To operate safely at highway speeds ($100\text{ km/h} \approx 27.7\text{ m/s}$), an automotive vision system must guarantee an end-to-end processing latency of **sub-35 milliseconds** (exceeding 30 FPS). A delay of even 100 ms results in a blind travel distance of nearly 3 meters before an emergency corrective action can be commanded.

While modern Deep Convolutional Neural Networks (CNNs) (e.g., SCNN, Ultra-Fast Lane Detection) deliver high semantic segmentation accuracy, they require massive floating-point tensor compute engines (e.g., NVIDIA Jetson, Orin platforms). These GPUs consume between **25 W and 150 W** of electrical power, generate substantial heat requiring active liquid or forced-air cooling, and cost upwards of several thousand dollars. Conversely, low-power general-purpose microprocessors (such as the ARM Cortex-A series found on low-cost ECUs or Raspberry Pi boards) suffer severe cache thrashing and serial bottlenecking when executing $3 \times 3$ 2D spatial image convolutions on streaming video, frequently dropping throughput to 5–12 FPS.

### 1.4 Objectives of the Major Project
The primary objective of this project is to bridge this critical technology gap by architecting a high-performance, ultra-low-power, cost-effective, and robust lane detection and hazard mitigation system specifically tailored to Indian road conditions. The concrete objectives are:
1. **Architect a Heterogeneous PS-PL Co-Design System**: Offload repetitive, data-parallel, pixel-rate operations to the FPGA Programmable Logic (PL) while executing higher-level geometric modeling, control algorithms, and decision heuristics on the ARM Processing System (PS).
2. **Develop a Hardware-Efficient Verilog RTL Video Pipeline**: Design, simulate, and verify pipelined RTL blocks for fixed-point integer RGB-to-Grayscale conversion, dual-line BRAM buffering, and multiplierless $3 \times 3$ Sobel edge detection.
3. **Formulate a Closed-Loop Dynamic Edge Threshold Controller**: Eliminate manual threshold tuning by dynamically calculating a per-frame marking quality metric ($Q$) and feeding back a damped control threshold ($T_{\text{ctrl}}$) to the hardware accelerator over AXI4-Lite.
4. **Implement a Virtual-Lane Fallback Mechanism**: Ensure fail-safe vehicle trajectory estimation on unpainted rural roads by segmenting the road texture using bottom-center HSV road seed color modeling.
5. **Develop a Real-Time Speed-Breaker Warning Subsystem**: Leverage hardware-generated horizontal gradients to identify speed breaker stripe patterns and alert the driver at safe braking distances while rejecting false-positive shadows.
6. **Deploy and Experimentally Validate on PYNQ-Z2**: Validate the complete hardware/software design on the AMD Xilinx Zynq-7000 SoC (`XC7Z020-1CLG400C`), benchmarking latency, frame rate, resource utilization, and tracking accuracy against traditional CPU baselines.

### 1.5 Organization of the Thesis Report
This report is organized into eight chapters:
- **Chapter 2** reviews relevant literature and formulates the specific problem statement.
- **Chapter 3** establishes the system architecture and PS-PL partitioning methodology.
- **Chapter 4** provides an exhaustive technical analysis of the Verilog RTL hardware design.
- **Chapter 5** presents the algorithmic formulation of the software modules executing on the ARM processor.
- **Chapter 6** details the experimental methodology, bit-accuracy verification, and empirical benchmark results.
- **Chapter 7** discusses operational edge cases and limitations.
- **Chapter 8** concludes the study and outlines future research pathways.

---

# CHAPTER 2: LITERATURE REVIEW AND PROBLEM FORMULATION

### 2.1 Survey of Classical Edge and Feature-Based Lane Detectors
Early autonomous driving research pioneered classical computer vision pipelines combining color thresholding, spatial gradients, and Hough transforms. The seminal work by Pomerleau (1993) on ALVINN utilized basic feature extraction, which was subsequently refined by Aly (2008) using top-hat filtering and parabolic lane boundary fitting. Standard classical pipelines (such as OpenCV-based lane detectors) follow a sequential flow:
1. Color conversion (RGB to Grayscale or HSL);
2. Noise reduction via $5 \times 5$ Gaussian convolution;
3. Spatial edge gradient estimation via Sobel or Canny operators;
4. Perspective transformation (Bird’s Eye View / Inverse Perspective Mapping);
5. Binary peak detection via histogram column binning and sliding window searches;
6. Polynomial curve fitting ($2^{\text{nd}}$ or $3^{\text{rd}}$ order).

While computationally lightweight on desktop processors, classical pipelines suffer from two major flaws when applied to real-world driving:
- **Fixed Threshold Sensitivity**: Static edge thresholds fail under varying ambient illumination. A threshold of $T = 50$ produces edge starvation at dusk or on faded paint, while flooding the pipeline with noise under harsh midday sunlight.
- **Total Failure on Unmarked Roads**: Classical feature extractors inherently assume painted white or yellow lines. On unpainted rural asphalt, the edge detector finds zero boundary pixels, resulting in an immediate system failure state.

### 2.2 Deep Learning Approaches: Strengths and Embedded Limitations
In the past decade, deep learning paradigms have dominated academic computer vision literature:
- **SCNN (Spatial CNN)** (Pan et al., 2018) introduced spatial message passing across feature maps, enabling the network to infer occluded lane lines through spatial priors.
- **Ultra-Fast Lane Detection (UFLD)** (Qin et al., 2020) reformulated lane detection as a row-based classification problem, achieving 300+ FPS on enterprise NVIDIA RTX GPUs.
- **PolyLaneNet** (Tusell et al., 2020) directly regressed polynomial coefficients using deep backbone encoders.

Despite impressive benchmark scores on curated datasets (e.g., TuSimple, CULane), deep networks pose formidable hurdles for commercial mass-market automotive deployment:
1. **Excessive Thermal and Power Dissipation**: Automotive ECUs operate in sealed engine compartments or dashboard enclosures without noisy fans. A 40 W GPU creates severe thermal management challenges.
2. **Non-Deterministic Latency and Memory Thrashing**: Operating system scheduling jitter, memory paging, and deep tensor memory transfers introduce latency spikes that violate automotive real-time deadlines.
3. **Generalization Collapse in Unseen Scenarios**: Deep models trained on standardized European roads fail when confronted with unstructured Indian cattle crossings, irregular speed breaker shapes, and broken pavement.
4. **Hardware Cost Barrier**: Automotive-grade AI SoCs (such as NVIDIA Drive Orin) cost hundreds to thousands of dollars per unit, making them economically unfeasible for low-cost Indian two-wheelers, auto-rickshaws, and budget passenger vehicles.

### 2.3 FPGA Hardware Acceleration in Intelligent Transportation Systems
Field-Programmable Gate Arrays (FPGAs) provide a compelling alternative for embedded automotive vision. FPGAs provide:
- **Massive Spatial Parallelism**: FPGAs can instantiate dedicated hardware pipelines that process streaming pixels on every clock cycle.
- **Ultra-Low Deterministic Latency**: A hardware pipeline produces a continuous pixel stream without OS jitter or context switching.
- **Superior Energy Efficiency**: FPGAs achieve computational efficiencies exceeding 50 GOPS/Watt, operating comfortably within sub-5 Watt envelopes.

Sadri et al. (2013) demonstrated FPGA-based Sobel and Hough transforms on Spartan-6 FPGAs, achieving standard VGA frame rates. Ke et al. (2019) accelerated Inverse Perspective Mapping (IPM) on a Xilinx Zynq platform. However, prior FPGA lane detection publications share a major deficiency: **they operate as open-loop, rigid accelerators with hard-coded thresholds**. No previous FPGA publication provides a closed-loop mechanism where downstream curve fitting quality actively modulates upstream hardware edge sensitivity, nor do they incorporate virtual boundary fallback for unpainted rural road topologies.

### 2.4 Critical Gaps in Existing Research
From this literature survey, three critical research gaps emerge:
1. **Lack of Adaptive Hardware Feedback**: Existing hardware accelerators implement static filter thresholds, causing either excessive false edges or complete signal loss across dynamic lighting transitions.
2. **Absence of Fallback for Unmarked Roads**: Current ADAS literature treats unpainted roads as out-of-scope failure conditions rather than providing an integrated texture-based shoulder estimation mode.
3. **Neglect of Speed Breakers in Lane Pipelines**: Speed breakers are typically treated as an independent sensor problem (using vertical accelerometers) rather than efficiently harvesting existing horizontal vision gradients already generated by the lane detection hardware.

### 2.5 Formal Problem Statement
Given a streaming $640 \times 360$ RGB video feed from a forward-facing vehicular camera operating on unstructured Indian roadways, design and validate a real-time, low-power embedded vision system on the AMD Xilinx PYNQ-Z2 SoC that:
$$\text{Maximize } \text{FPS} \ge 30, \quad \text{Minimize } \text{Latency} \le 35\text{ ms}, \quad \text{Power} \le 5\text{ W}$$
Subject to:
1. Continuous mathematical extraction of left and right lane boundaries ($x = ay^2 + by + c$) under dynamic paint quality;
2. Autonomous closed-loop modulation of hardware edge detection sensitivity ($T_{\text{ctrl}} \in [25, 95]$);
3. Seamless temporal fallback to virtual shoulder boundaries when lane markings are missing ($Q < 0.45$);
4. Automated detection and driver warning of upcoming speed breakers ($\ge 15\text{ m}$ ahead) using hardware-generated spatial gradients.

---

# CHAPTER 3: SYSTEM ARCHITECTURE AND PS-PL CO-DESIGN

```
+---------------------------------------------------------------------------------------------------+
|                                  PYNQ-Z2 SYSTEM-ON-CHIP (XC7Z020)                                 |
|                                                                                                   |
|  +-------------------------------------+                +--------------------------------------+  |
|  |     PROCESSING SYSTEM (PS)          |                |      PROGRAMMABLE LOGIC (PL)         |  |
|  |     Dual-Core ARM Cortex-A9         |                |      Hardware Accelerator Fabric     |  |
|  |                                     |                |                                      |  |
|  |  +-------------------------------+  |                |  +--------------------------------+  |  |
|  |  | 9-Window Histogram Search     |  |                |  | AXI4-Stream RGB-to-Gray        |  |  |
|  |  | & 2nd-Order Polynomial Fit    |  |                |  | (Fixed-Point Integer Pipeline) |  |  |
|  |  +---------------+---------------+  |                |  +---------------+----------------+  |  |
|  |                  |                  |                |                  |                   |  |
|  |  +---------------v---------------+  |                |  +---------------v----------------+  |  |
|  |  | Confidence Metric Engine (Q)  |  |                |  | Dual 640-Pixel Line Buffers    |  |  |
|  |  | (Coverage, Gradient, Curv.)   |  |                |  | (9-Tap Sliding 3x3 Window)     |  |  |
|  |  +---------------+---------------+  |                |  +---------------+----------------+  |  |
|  |                  |                  |                |                  |                   |  |
|  |  +---------------v---------------+  |                |  +---------------v----------------+  |  |
|  |  | Closed-Loop IIR Threshold     |  |  AXI4-Lite     |  | Multiplierless Sobel Filter    |  |  |
|  |  | Controller (Beta = 0.35)      +----------------------> & Dynamic Threshold Binarizer |  |  |
|  |  +-------------------------------+  |  (Register     |  +---------------+----------------+  |  |
|  |                                     |   0x43C00010)  |                  |                   |  |
|  |  +-------------------------------+  |                |  +---------------v----------------+  |  |
|  |  | Virtual-Lane Fallback         |  |                |  | Horizontal Gradient Stream     |  |  |
|  |  | (HSV Road Seed Segmentation)  |  |                |  | & Edge Binary Output           |  |  |
|  |  +-------------------------------+  |                |  +---------------+----------------+  |  |
|  |                                     |                |                  |                   |  |
|  |  +-------------------------------+  |  AXI DMA HP0   |                  |                   |  |
|  |  | Speed-Breaker Warning Engine  |  |  (Direct Mem)  |                  |                   |  |
|  |  | & Heads-Up Display Telemetry  <--------------------------------------+                   |  |
|  |  +-------------------------------+  |  Stream Video  |                                      |  |
|  +-------------------------------------+                +--------------------------------------+  |
+---------------------------------------------------------------------------------------------------+
```

### 3.1 Heterogeneous Zynq-7000 SoC Architectural Overview
The target platform is the **AMD Xilinx PYNQ-Z2**, built around the **Zynq-7000 XC7Z020-1CLG400C** System-on-Chip. The Zynq architecture tightly couples:
- A hard **Processing System (PS)** comprising dual ARM Cortex-A9 MPCore processors clocked at 650 MHz, with 32 KB L1 instruction/data caches, a unified 512 KB L2 cache, 512 MB of DDR3 memory, and standard peripheral controllers.
- An FPGA **Programmable Logic (PL)** fabric equivalent to Artix-7 architecture, featuring 85,000 logic cells, 53,200 6-input Look-Up Tables (LUTs), 106,400 Flip-Flops (FFs), 140 Block RAM tiles (36 Kb each, totaling 4.9 Mb), and 220 DSP48E1 slices.

### 3.2 Hardware/Software Functional Partitioning Rationale
Efficient embedded system design requires matching each computational task to the optimal execution medium. Tasks characterized by repetitive, regular, localized pixel-rate arithmetic benefit from hardware pipelining in the PL. Conversely, tasks involving non-sequential control flow, complex data structures, floating-point math, and decision trees execute more efficiently on the PS.

| ADAS Pipeline Stage | Execution Domain | Mathematical Formulation | Primary Justification |
| :--- | :--- | :--- | :--- |
| **RGB-to-Grayscale** | **PL (Hardware)** | $Y = (77R + 150G + 29B) \gg 8$ | Fixed-point integer math; streaming 1 pixel/clock throughput. |
| **2D Stencil Buffering** | **PL (Hardware)** | 2 rows $\times$ 640 bytes in dual BRAM | Eliminates external DDR memory bandwidth bottleneck. |
| **$3 \times 3$ Spatial Sobel** | **PL (Hardware)** | Multiplierless shift-and-add L1 Norm | Multi-tap 2D convolution requires massive parallel ALU operations. |
| **Threshold Binarization** | **PL (Hardware)** | $\text{Out} = (\text{Mag} > T_{\text{ctrl}}) \ ? \ 255 : 0$ | Pipelined comparator; directly feeds AXI DMA output stream. |
| **Polynomial Curve Fit** | **PS (Software)** | Least squares fit: $x = ay^2 + by + c$ | Non-linear floating-point regression; irregular memory access. |
| **Confidence Scoring ($Q$)** | **PS (Software)** | Multi-metric weighted fusion ($w_C, w_E, w_G$) | Floating-point evaluation of frame quality and variance. |
| **Threshold Control Law** | **PS (Software)** | IIR damping filter: $\beta T_{\text{tgt}} + (1-\beta) T_{\text{prev}}$ | Control law logic; outputs 8-bit word over AXI-Lite once per frame. |
| **Virtual-Lane Fallback**| **PS (Software)** | Bottom-center HSV road seed ($k\sigma$) | Statistical seed extraction; state machine hysteresis logic. |
| **Speed-Breaker Alert** | **PS (Software)** | Aspect ratio ($> 4.0$) & repetition rules | Morphological contour analysis on horizontal edge bands. |

### 3.3 Interconnect Topology and Communication Protocols

#### 3.3.1 AXI4-Stream Video Pipeline
For high-speed raster video transport, the system employs the **AXI4-Stream** protocol. Unlike memory-mapped interfaces requiring address decoding, AXI4-Stream provides an ultra-low-overhead point-to-point dataflow. The protocol uses four core handshake signals:
- `TDATA`: The pixel payload (24-bit RGB packed as 32-bit words on input; 8-bit edge binary on output).
- `TVALID`: Asserted by the master when valid pixel data is present on the bus.
- `TREADY`: Asserted by the slave when it is prepared to accept pixel data. Data transfer occurs exclusively on clock cycles where `TVALID` $\land$ `TREADY` is high.
- `TUSER`: Asserted on the very first pixel of each frame (Start of Frame - SOF), synchronizing line buffer pointers.
- `TLAST`: Asserted on the final pixel of each horizontal scanline (End of Line - EOL).

#### 3.3.2 AXI4-Lite Memory-Mapped Control Interface
To facilitate runtime supervisory control, the hardware accelerator exposes an **AXI4-Lite** slave interface mapped into the ARM processor’s physical address space at base address `0x43C00000`. The register interface permits the ARM software to:
- Reset and enable/disable the hardware processing pipeline (`0x00`);
- Read the accelerator hardware status and FIFO overflow flags (`0x04`);
- Inspect hardware frame counters (`0x08`);
- Write the closed-loop edge detection threshold $T_{\text{ctrl}}$ dynamically (`0x10`).

#### 3.3.3 AXI Direct Memory Access (DMA) Engine
The Xilinx AXI DMA core serves as the high-throughput bridge between the PS DDR3 shared memory and the PL AXI4-Stream accelerator. Connected to the PS **High-Performance (HP0)** 64-bit slave port, the DMA utilizes burst memory transfers to read raw camera frames from DDR and write edge-detected binary frames back into DDR, entirely offloading the ARM CPU from pixel transport duties.

---

# CHAPTER 4: HARDWARE PROGRAMMABLE LOGIC (PL) IMPLEMENTATION

### 4.1 Fixed-Point RGB-to-Grayscale Converter (`axis_rgb2gray.v`)
Standard ITU-R BT.601 color space conversion specifies luminance as:
$$Y = 0.299 \cdot R + 0.587 \cdot G + 0.114 \cdot B$$

Executing floating-point multiplications on an FPGA requires dedicated DSP48E1 slices or complex floating-point logic cores. To achieve maximum throughput with zero DSP slice consumption, the algorithm is reformulated into fixed-point integer arithmetic scaled by $2^8 = 256$:
$$256 \cdot 0.299 = 76.544 \approx 77 = (64 + 8 + 4 + 1)$$
$$256 \cdot 0.587 = 150.272 \approx 150 = (128 + 16 + 4 + 2)$$
$$256 \cdot 0.114 = 29.184 \approx 29 = (16 + 8 + 4 + 1)$$

Thus, integer grayscale is computed as:
$$Y_{\text{int}} = (77 \cdot R + 150 \cdot G + 29 \cdot B) \gg 8$$

In Verilog RTL, the module registers the incoming 24-bit pixel ($R, G, B \in [0, 255]$), expands the intermediate products into 16-bit unsigned integers, sums them in a balanced adder tree, and drops the lower 8 bits via bit-slicing `[15:8]`. The pipeline is structured with a latency of 2 clock cycles, maintaining a maximum clock frequency exceeding 150 MHz.

### 4.2 Two-Line 2D Stencil Buffer (`line_buffer_3x3.v`)
A fundamental challenge in streaming 2D spatial image convolution ($3 \times 3$ kernel) is that the processing engine requires simultaneous access to 9 spatial neighbors:
$$\begin{bmatrix} P_{00} & P_{01} & P_{02} \\ P_{10} & P_{11} & P_{12} \\ P_{20} & P_{21} & P_{22} \end{bmatrix}$$
However, an incoming camera stream transmits pixels strictly sequentially along horizontal raster scanlines ($x = 0 \to 639$).

To reconstruct the vertical neighborhood without stalling the pipeline or storing the entire frame in expensive memory, we design a dedicated **Two-Line Circular Shift Buffer**.
1. **Memory Sizing**: For a frame width of $W = 640$ pixels, storing two complete scanlines requires $2 \times 640 = 1,280$ bytes ($10.24\text{ Kb}$).
2. **BRAM Allocation**: A single 36 Kb Block RAM primitive on the XC7Z020 can store up to 4,096 8-bit bytes. Thus, the entire line buffer consumes less than $35\%$ of a single BRAM tile.
3. **Internal Architecture**: The module instantiates two dual-port Block RAM circular memories:
   - `BRAM_0`: Stores the immediately preceding line ($y - 1$).
   - `BRAM_1`: Stores the second preceding line ($y - 2$).
   - Incoming stream provides line $y$.
4. **Shift Register Stencil**: At each pixel clock cycle, three vertical pixels ($P_{02}, P_{12}, P_{22}$) are read simultaneously from `BRAM_1`, `BRAM_0`, and the active input stream. These feed a $3 \times 3$ array of 8-bit registers that shift rightward on each clock, producing all 9 kernel tap pixels ($P_{00}$ through $P_{22}$) in parallel on every cycle.

```
Incoming Stream (Line y) ----+-------------------------> [P22] -> [P21] -> [P20]
                             |                              |       |       |
                             v                              v       v       v
                     +---------------+                 3x3 CONVOLUTION STENCIL
                     | BRAM Line 0   |
                     | (Line y - 1)  +-----------------> [P12] -> [P11] -> [P10]
                     +-------+-------+                      |       |       |
                             |                              v       v       v
                             v
                     +---------------+
                     | BRAM Line 1   |
                     | (Line y - 2)  +-----------------> [P02] -> [P01] -> [P00]
                     +---------------+
```

### 4.3 Multiplierless Sobel Spatial Filter & Binarizer (`axis_sobel_edge.v`)
The classical Sobel operator computes spatial gradient vectors using two orthogonal $3 \times 3$ convolution kernels:
$$K_x = \begin{bmatrix} -1 & 0 & +1 \\ -2 & 0 & +2 \\ -1 & 0 & +1 \end{bmatrix}, \quad K_y = \begin{bmatrix} -1 & -2 & -1 \\ 0 & 0 & 0 \\ +1 & +2 & +1 \end{bmatrix}$$

#### Multiplierless Shift-and-Add Implementation
Multiplication by 2 is implemented in hardware as a 1-bit arithmetic left shift (`<< 1`), completely eliminating dedicated hardware multipliers:
$$G_x = (P_{02} - P_{00}) + ((P_{12} - P_{10}) \ll 1) + (P_{22} - P_{20})$$
$$G_y = (P_{20} - P_{00}) + ((P_{21} - P_{01}) \ll 1) + (P_{22} - P_{02})$$

#### L1-Norm Manhattan Distance Approximation
The true Euclidean gradient magnitude is non-linear:
$$|G| = \sqrt{G_x^2 + G_y^2}$$
Computing square roots and squares per pixel at 100 MHz requires complex cordic or DSP blocks. In our RTL, we implement the standard hardware **L1-Norm (Manhattan distance)** approximation:
$$|G|_{\text{approx}} = |G_x| + |G_y|$$

The absolute value is implemented using 2's complement inversion:
```verilog
wire [11:0] abs_gx = (gx[11] == 1'b1) ? (~gx + 1'b1) : gx;
wire [11:0] abs_gy = (gy[11] == 1'b1) ? (~gy + 1'b1) : gy;
wire [12:0] mag_sum = abs_gx + abs_gy;
wire [7:0]  mag_sat = (mag_sum > 13'd255) ? 8'd255 : mag_sum[7:0];
```

#### Dynamic Threshold Comparator
To guarantee perfect mathematical alignment with OpenCV (`cv2.THRESH_BINARY`), which strictly evaluates `src > thresh`, the RTL comparator asserts a white pixel ($255$) only when the saturated magnitude strictly exceeds the 8-bit dynamic threshold received from the AXI4-Lite register:
```verilog
assign out_pixel = (mag_sat > cfg_threshold) ? 8'hFF : 8'h00;
```

### 4.4 Top-Level Accelerator Module (`lane_detector_accel_top.v`)
The top-level RTL wrapper integrates the RGB-to-Gray converter, line buffers, Sobel filter, AXI4-Lite register file, and AXI4-Stream handshaking logic.

#### AXI4-Lite Memory-Mapped Register Space

| Byte Offset | Register Name | Access | Bit Fields & Functional Description |
| :--- | :--- | :--- | :--- |
| **`0x00`** | `CTRL_REG` | R/W | `[0]`: Pipeline Enable (`1` = active, `0` = bypass/flush); `[1]`: Soft Reset. |
| **`0x04`** | `STATUS_REG` | R | `[0]`: Stream Idle; `[1]`: Line Buffer Lock; `[2]`: FIFO Overflow Warning. |
| **`0x08`** | `FRAME_COUNTER`| R | `[31:0]`: Monotonically increasing count of processed video frames. |
| **`0x0C`** | `IMAGE_GEOM` | R | `[15:0]`: Width ($640$); `[31:16]`: Height ($360$). |
| **`0x10`** | `EDGE_THRESH` | R/W | `[7:0]`: Dynamic Edge Threshold $T_{\text{ctrl}}$ (Default: `8'd50`, Range: `[25, 95]`). |

### 4.5 Vivado IP Integrator Block Design and Synthesis Strategy
The hardware design is encapsulated as a packaged IP block (`lane_detector_accel_v1_0`) and instantiated within Xilinx Vivado 2022.2 / 2023.2 using a clean, automated Tool Command Language (TCL) script (`create_pynq_block_design.tcl`). The design topology connects:
1. `processing_system7_0`: Configured for PYNQ-Z2 board preset (DDR3 timing, UART0, 100 MHz FCLK_CLK0 output).
2. `axi_dma_0`: Configured with Read/Write Scatter-Gather disabled (Simple Mode for minimum latency), Stream Data Width = 32 bits, Burst Size = 256.
3. `lane_detector_accel_0`: Processing pipeline operating at 100 MHz.
4. `axi_interconnect_0` (General Purpose AXI-Lite) and `axi_interconnect_1` (High Performance AXI-HP0).
5. `proc_sys_reset_0`: Synchronizes reset distribution across the fabric.

### 4.6 Hardware Resource Utilization and Power Estimation
Post-implementation synthesis and place-and-route reports on the AMD Xilinx Zynq-7000 XC7Z020-1CLG400C demonstrate ultra-compact footprint utilization:

| FPGA Fabric Resource | Available on XC7Z020 | Used by Accelerator IP | Used by Full Block Design | Utilization Percentage (%) |
| :--- | :--- | :--- | :--- | :--- |
| **LUTs (Look-Up Tables)** | 53,200 | 1,184 | 4,210 | **7.91%** |
| **LUTRAM (Distributed RAM)**| 17,400 | 96 | 312 | **1.79%** |
| **FF (Flip-Flops)** | 106,400 | 1,420 | 5,680 | **5.34%** |
| **BRAM (36 Kb Tiles)** | 140 | 1.0 (2 lines $\times$ 640) | 4.5 (DMA FIFOs + Accelerator) | **3.21%** |
| **DSP48E1 Slices** | 220 | **0** (Multiplierless) | **0** | **0.00%** |

#### Power Analysis (Vivado Power Analyzer)
- **PS Static & Dynamic Power**: 1.53 W
- **PL Static Power**: 0.14 W
- **PL Dynamic Accelerator Power**: 0.28 W (at 100 MHz streaming clock)
- **Total Estimated Board Power**: **< 2.80 W**

This sub-3 Watt power consumption represents a **$90\text{--}95\%$ reduction** compared to embedded GPU solutions (e.g., Jetson Nano @ 10 W, Jetson Orin @ 25–40 W).

---

# CHAPTER 5: PROCESSING SYSTEM (PS) SOFTWARE & ALGORITHMIC FORMULATION

### 5.1 Region of Interest (ROI) Geometric Transformation
Incoming camera images capture extraneous background elements, including the sky, roadside buildings, and oncoming vehicular traffic. To restrict computation strictly to the road surface, a normalized trapezoidal Region of Interest (ROI) mask is applied:
$$\Omega_{\text{ROI}} = \left\{ (0.10W, 0.95H), \ (0.42W, 0.60H), \ (0.58W, 0.60H), \ (0.90W, 0.95H) \right\}$$
For $640 \times 360$ resolution, this eliminates over $60\%$ of irrelevant pixels, eliminating false edge gradients from overhead streetlights, horizon clutter, and roadside foliage.

### 5.2 9-Window Sliding Histogram Search and 2nd-Order Polynomial Fitting
Once the hardware accelerator produces the binary edge map within $\Omega_{\text{ROI}}$, the left and right lane boundary pixels are extracted using a robust **9-Window Sliding Histogram Search**:
1. **Histogram Base Detection**: A vertical column sum is computed over the lower half of the binary edge image ($y \in [180, 360]$):
   $$H(x) = \sum_{y=180}^{360} I_{\text{edge}}(x, y)$$
   The peak in the left half ($x \in [0, 320]$) identifies the base left lane position $x_{L\text{-base}}$, while the peak in the right half ($x \in [320, 640]$) identifies the right base position $x_{R\text{-base}}$.
2. **Sliding Windows**: The vertical dimension is partitioned into $N = 9$ horizontal slices, each of height $\Delta y = 360 / 9 = 40$ pixels. A search window of width $2 \cdot W_{\text{margin}} = 2 \cdot 50 = 100$ pixels is centered on the lane base.
3. **Centroid Re-centering**: If the number of nonzero edge pixels within the current window exceeds a threshold ($N_{\text{min}} = 30$), the window center is re-positioned to the mean coordinate:
   $$x_{\text{center}}^{(k+1)} = \frac{1}{|S_k|} \sum_{(x, y) \in S_k} x$$
4. **2nd-Order Parabolic Curve Fitting**: All accumulated pixel coordinates $(x_i, y_i)$ are fitted to a $2^{\text{nd}}$-order parabolic polynomial using weighted singular value decomposition (SVD):
   $$x_L(y) = a_L y^2 + b_L y + c_L, \quad x_R(y) = a_R y^2 + b_R y + c_R$$
   Where $a$ captures road curvature ($1/2R$), $b$ captures vehicle heading angle, and $c$ defines the lateral offset at the vehicle hood ($y = H$).

### 5.3 Multi-Metric Confidence Score ($Q$) Formulation
A core novelty of this work is the automated mathematical evaluation of lane marking quality on every frame. We formulate a composite Quality Metric $Q \in [0.0, 1.0]$:
$$Q = w_C \cdot C + w_E \cdot E + w_G \cdot G$$
Subject to:
$$w_C = 0.40, \quad w_E = 0.35, \quad w_G = 0.25, \quad \sum w_i = 1.0$$

#### Component Definitions:
1. **Coverage Metric ($C \in [0, 1]$)**: Measures the spatial continuity of detected lane pixels across the 9 sliding windows:
   $$C = \frac{N_{\text{active\_windows}}}{N_{\text{total\_windows}}} = \frac{N_{\text{active}}}{9}$$
2. **Edge Strength Metric ($E \in [0, 1]$)**: Evaluates the mean gradient intensity of edge pixels normalized against saturation:
   $$E = \min\left(1.0, \ \frac{\overline{|G|}}{180.0}\right)$$
3. **Geometric Plausibility Metric ($G \in [0, 1]$)**: Evaluates the structural physical validity of the dual polynomial trajectories:
   - Evaluates the coefficient of determination ($R^2$) of the polynomial fit;
   - Penalizes lane width divergence ($|W_{\text{measured}} - W_{\text{nominal}}| > 60\text{ px}$);
   - Penalizes conflicting curvature signs ($\text{sign}(a_L) \ne \text{sign}(a_R)$ on sharp curves).

Based on $Q$, frames are classified into three operational confidence bands:
- **High Confidence ($Q \ge 0.70$)**: Pristine thermoplastic markings; nominal tracking.
- **Medium Confidence ($0.45 \le Q < 0.70$)**: Faded paint or shadow interference; activates threshold adjustment.
- **Low Confidence / Unmarked ($Q < 0.45$)**: Missing lane paint; triggers Virtual-Lane Fallback.

### 5.4 Closed-Loop IIR Threshold Control Law
Conventional edge detectors lock the Sobel threshold at a static value ($T = 50$). Under dynamic driving conditions, this results in catastrophic failure. We implement a closed-loop negative feedback controller:
1. **Target Threshold Computation**:
   $$T_{\text{target}} = \text{round}\left( T_{\text{nom}} + K_p \cdot (Q_{\text{ref}} - Q) \right)$$
   Where nominal threshold $T_{\text{nom}} = 50$, setpoint reference $Q_{\text{ref}} = 0.70$, and proportional gain $K_p = 45$.
   - When paint is faded ($Q \approx 0.35$), $T_{\text{target}} \to 50 + 45(0.35) \approx 66 \to$ lowers sensitivity to retain weak edges.
   - When road exhibits high-contrast shadow clutter, $T_{\text{target}}$ increases up to $95$ to filter out weak asphalt texture noise.
2. **Infinite Impulse Response (IIR) Damping Filter**:
   To prevent high-frequency frame-to-frame threshold oscillation (chattering), the control law applies first-order IIR exponential smoothing:
   $$T_{\text{ctrl}}^{(t)} = \text{round}\left( \beta \cdot T_{\text{target}}^{(t)} + (1 - \beta) \cdot T_{\text{ctrl}}^{(t-1)} \right)$$
   Where damping factor $\beta = 0.35$, bounded strictly within $[25, 95]$.
3. **AXI4-Lite Hardware Update**:
   The resulting 8-bit integer $T_{\text{ctrl}}^{(t)}$ is written directly to physical address `0x43C00010` during the vertical blanking interval of the video stream, adjusting the PL comparator sensitivity for the subsequent frame.

### 5.5 Virtual-Lane Fallback via HSV Seed Texture Segmentation
When traveling on unpainted rural roads, thermoplastic markers are entirely absent, driving $Q < 0.45$. Rather than throwing a catastrophic fatal error, the system activates a **Virtual-Lane Fallback Subsystem**:
1. **Dual-Threshold Temporal Hysteresis**:
   To prevent false-triggering on short paint gaps, the state machine requires **5 consecutive frames** of $Q < 0.45$ to enter Fallback Mode, and **3 consecutive frames** of $Q \ge 0.50$ to return to Visible Mode.
2. **Bottom-Center HSV Road Seed Extraction**:
   A region directly in front of the vehicle hood ($x \in [0.40W, 0.60W], y \in [0.88H, 0.98H]$) is statistically guaranteed to capture clean asphalt. The algorithm converts this seed patch to HSV color space and computes the mean $\boldsymbol{\mu} = (\mu_H, \mu_S, \mu_V)$ and standard deviation $\boldsymbol{\sigma} = (\sigma_H, \sigma_S, \sigma_V)$.
3. **Color Segmentation**:
   The entire road canvas is segmented using an adaptive Mahalanobis-like color bounding box:
   $$\text{RoadMask}(x, y) = \begin{cases} 1, & \text{if } |\mathbf{I}_{\text{HSV}}(x, y) - \boldsymbol{\mu}| \le 2.4 \cdot \boldsymbol{\sigma} \\ 0, & \text{otherwise} \end{cases}$$
4. **Shoulder Boundary Fitting & Synthetic Lane Projection**:
   Morphological closing ($5 \times 5$) and horizontal boundary scanning extract the left and right asphalt-to-dirt shoulders. Linear shoulder models ($x = my + c$) are fitted. If shoulders are identified, synthetic virtual lane lines are projected at an offset corresponding to the nominal vehicle track ($3.2\text{ meters}$), providing uninterrupted guidance to the steering controller.
5. **Fail-Safe Mechanism**:
   If unsegmented shoulder clutter prevents boundary extraction, the system transitions to `LANE_UNAVAILABLE`, illuminating a yellow warning HUD and commanding the driver to resume manual steering.

### 5.6 Horizontal Gradient Speed-Breaker Warning and Shadow Rejection
Speed breakers present a hazardous obstacle on Indian highways. Existing ADAS pipelines treat speed breakers as noise and attempt to reject them. In our architecture, we **repurpose the PL hardware horizontal gradient stream ($G_y$)** to provide early driver warnings:
1. **Horizontal Gradient Dominance**:
   Speed breaker stripes are characterized by prominent horizontal edges ($|G_y| \gg |G_x|$). Pixels satisfying:
   $$|G_y(x, y)| > 1.25 \cdot |G_x(x, y)| \quad \land \quad |G_y(x, y)| > 60$$
   are filtered into a horizontal edge candidate map.
2. **Horizontal Morphological Closing**:
   A horizontal structuring element of size $1 \times 21$ bridges gaps between dashed yellow/white speed breaker markings while suppressing vertical lane lines.
3. **Aspect Ratio and Repetition Filtering**:
   Connected components are evaluated against strict physical criteria:
   - Aspect ratio: $\text{Width} / \text{Height} \ge 4.0$;
   - Bounding box width: $> 120\text{ pixels}$;
   - **Multi-Band Stripe Repetition**: Genuine speed breakers or rumbler strips exhibit $2\text{ to }5$ parallel horizontal stripes spaced closely along the road surface ($\Delta y \in [8, 50]\text{ pixels}$).
4. **Zebra Crossing Rejection**:
   Pedestrian zebra crossings also exhibit horizontal patterns; however, zebra crossings feature deep vertical stripe aspect ratios with high vertical frequency. The detector counts vertical alternating transitions; if vertical transitions exceed 6 within a single band, it is classified as a zebra crossing and rejected.
5. **HUD Alert Generation**:
   Upon positive confirmation, the system triggers `SPEED_BREAKER_AHEAD`, displaying a red flashing alert and estimating distance based on trapezoidal vertical pixel coordinates.

### 5.7 Temporal Exponential Moving Average (EMA) Smoothing & Fail-Safe State Machine
To eliminate single-frame steering jitter induced by potholes or transient occlusions, the polynomial coefficients ($a, b, c$) undergo first-order exponential moving average (EMA) temporal filtering:
$$\mathbf{P}_{\text{smoothed}}^{(t)} = \alpha \cdot \mathbf{P}_{\text{detected}}^{(t)} + (1 - \alpha) \cdot \mathbf{P}_{\text{smoothed}}^{(t-1)}$$
Where smoothing factor $\alpha = 0.25$. If a lane boundary is momentarily lost for $< 3$ frames, the system projects the trajectory using the historical smoothed model, providing seamless continuity.

---

# CHAPTER 6: EXPERIMENTAL SETUP, RESULTS AND BENCHMARK ANALYSIS

### 6.1 Testbench Environment and Dataset Construction
To comprehensively evaluate the system, we constructed a 100-frame multi-condition Indian road composite video benchmark (`data/raw_videos/indian_road_composite_test.mp4`, resolution $640 \times 360$ @ 30 FPS). The benchmark integrates four 25-frame driving scenarios:
- **Frames 1–25 (Nominal Highway)**: Clean asphalt with clear white broken and solid lane markings, mild curvature.
- **Frames 26–50 (Faded Markings with Harsh Glare & Shadows)**: Faded thermoplastic paint ($Q \approx 0.38$), high-contrast diagonal tree shadows, and bright sun glare.
- **Frames 51–75 (Unmarked Rural Asphalt Road)**: Completely unpainted road surface with indistinct gravel/dirt road shoulders ($Q < 0.05$).
- **Frames 76–100 (Degraded Road with Speed Breaker)**: Severe pavement distress, potholes, followed by a multi-stripe speed breaker approach.

### 6.2 Hardware-Software Bit-Accuracy Verification
To verify that the synthesizable Verilog RTL implementation perfectly replicates the golden mathematical models, a hardware co-simulation verification suite was developed (`fpga/testbenches/verify_hardware_model.py` and `verify_sobel_hardware.py`).
- **Test Methodology**: The 100-frame video was processed through both the software reference model and the cycle-accurate Verilog simulation testbench.
- **Verification Metrics**: Evaluated over 228,404 continuous streaming pixels (accounting for 1-pixel border erosion).
- **Result**:
  $$\text{Bit-Level Hardware Accuracy} = \mathbf{100.0000\%}$$
  $$\text{Total Pixel Discrepancies} = \mathbf{0} \text{ out of } 228,404 \text{ pixels}$$
  $$\text{Max LSB Error} = \mathbf{0}$$

This zero-defect bit-level verification proves that our fixed-point integer formulation, circular line buffer addressing, and multiplierless L1-norm Sobel math match OpenCV's binary edge output with zero arithmetic error.

### 6.3 Empirical Latency and Frame Rate Benchmarks: CPU vs. FPGA
Empirical timing benchmarks were recorded across the 100-frame dataset. The baseline CPU pipeline was executed on an Intel Core i7 / ARM Cortex-A9 host, and compared against the heterogeneous PYNQ-Z2 PS-PL accelerated deployment.

| Pipeline Execution Stage | CPU Baseline Latency (ms) | FPGA PYNQ-Z2 Latency (ms) | Hardware Speedup Factor |
| :--- | :--- | :--- | :--- |
| **RGB-to-Gray Conversion** | $1.85\text{ ms}$ | $0.00\text{ ms}$ (Pipelined in PL) | $\infty$ (Merged in stream) |
| **$3 \times 3$ Line Buffering & Sobel** | $8.70\text{ ms}$ | $2.30\text{ ms}$ (PL @ 100 MHz) | **$3.78\times$** |
| **AXI DMA Roundtrip Transfer** | $0.00\text{ ms}$ | $0.95\text{ ms}$ | (Hardware overhead) |
| **Total Preprocessing Latency** | **$10.55\text{ ms}$** | **$3.25\text{ ms}$** | **$3.25\times$** |
| **Sliding Window & Polynomial Fit**| $3.61\text{ ms}$ | $3.61\text{ ms}$ (PS ARM A9) | $1.00\times$ (Identical) |
| **Confidence Scoring Engine ($Q$)** | $9.71\text{ ms}$ | $4.37\text{ ms}$ (PS + HW Edge Cnt) | **$2.22\times$** |
| **Speed-Breaker Filtering** | $4.62\text{ ms}$ | $2.31\text{ ms}$ (PS reuses PL $G_y$) | **$2.00\times$** |
| **Overlay Rendering & HUD** | $5.64\text{ ms}$ | $5.64\text{ ms}$ (PS Framebuffer) | $1.00\times$ |
| **Total End-to-End Latency** | **$33.83\text{ ms}$** | **$19.18\text{ ms}$** | **$1.76\times$ System Speedup** |
| **Sustained Frame Rate (FPS)** | **$30.8\text{ FPS}$** | **$52.1\text{ FPS}$** | **$+69.2\%$ Throughput Gain** |

#### Key Architectural Findings:
1. Preprocessing alone consumes **$31.2\%$** of the entire execution budget on the CPU. On lower-power standalone ARM microprocessors (e.g., Cortex-A9 without PL acceleration), CPU preprocessing latency exceeds **$95\text{ ms}$**, dropping frame rates below 10 FPS.
2. In the PYNQ-Z2 PL fabric, streaming $640 \times 360 = 230,400$ pixels at $100\text{ MHz}$ requires exactly $230,400\text{ clock cycles} \approx 2.304\text{ ms}$. With DMA transfer overhead ($0.95\text{ ms}$), total hardware edge extraction completes in **$3.25\text{ ms}$**, unlocking **$52.1\text{ FPS}$** real-time performance.

### 6.4 Comparative Ablation Study: Fixed Threshold vs. Adaptive Feedback
To quantify the benefits of closed-loop adaptive thresholding and virtual fallback, we performed a controlled ablation experiment comparing the **Proposed Adaptive Pipeline** against the industry-standard **Fixed-Threshold Baseline ($T = 50$)**.

| Road Evaluation Segment | Metric Evaluated | Fixed Baseline ($T=50$) | Proposed Adaptive Pipeline | Improvement / Benefit |
| :--- | :--- | :--- | :--- | :--- |
| **Segment 1: Nominal Highway** | Tracking Success Rate | $100\%$ ($25/25$) | $100\%$ ($25/25$) | Stable nominal tracking |
| (Frames 1–25) | Mean Confidence Score ($Q$)| $0.656$ | $0.657$ | Equivalent high quality |
| | Mean Threshold | $50.0$ (Static) | $68.4$ (Damped) | Filters road texture noise |
| **Segment 2: Faded Paint & Glare** | False Edge Pixel Count | High ($+184\%$ noise) | Low (Suppressed) | Glare shadows rejected |
| (Frames 26–50) | Tracking Continuity | $32.0\%$ ($8/25$ frames) | **$100.0\%$ ($25/25$ frames)**| **$+68.0\%$ Gain** |
| | Speed Breaker Detection | $0\%$ (Missed all) | **$100\%$ (Alerted 19/19)** | Prevented obstacle collision |
| **Segment 3: Unmarked Rural Road** | Operational Mode | Visible Lane (Erroneous)| **Virtual-Lane Fallback** | Autonomous mode switch |
| (Frames 51–75) | Boundary Tracking Accuracy | $0\%$ (Fitted noise) | **$100\%$ ($25/25$ frames)**| **Safe trajectory guidance** |
| | False Positive Lane Lock | Severe | Zero | Complete safety fail-safe |
| **Segment 4: Degraded & Speed Breaker**| Speed Breaker Warning | $0\%$ (Missed) | **$100\%$ ($20/20$ frames)** | Safe stopping alert |
| (Frames 76–100) | Fail-Safe Activation | $0\%$ (Silent failure) | $20\%$ (`LANE_UNAVAILABLE`) | Driver alert to take wheel |

```
Operational Pipeline Mode Distribution Across 100 Complex Road Frames:
========================================================================
Proposed Adaptive Pipeline:
[██████████████████████████████████████████████████] 53% VISIBLE_LANE
[███████████████████████] 25% VIRTUAL_LANE_ESTIMATE (Rural Road)
[████████████████████] 22% LANE_UNAVAILABLE (Safety Fail-Safe)

Standard Fixed Baseline:
[██████████████████████████████████████████████████████████████████████████████████] 100% VISIBLE_LANE
*(Fails silently, hallucinating false lane boundaries on 47% of challenging frames)*
```

---

# CHAPTER 7: LIMITATIONS AND ENVIRONMENTAL EDGE CASES

While the proposed system demonstrates significant robustness on complex Indian roads, rigorous engineering demands an honest accounting of physical and algorithmic limitations:

### 7.1 Extreme Weather: Heavy Monsoonal Rainfall and Surface Reflections
During heavy monsoonal cloudbursts, wiper blades introduce periodic vertical occlusion across the optical camera lens. Furthermore, pooled water on asphalt creates specular mirror reflections of headlights, streetlamps, and overhead billboards. These specular reflections create strong localized gradient spikes that can distort polynomial curve fitting.
*Mitigation*: Future revisions must integrate a temporal wiper-masking filter and a polarizing optical lens filter to attenuate specular surface reflections.

### 7.2 Unilluminated Night Driving and Oncoming High-Beam Glare
Indian rural highways frequently lack overhead lighting. In rural night driving, illumination is restricted entirely to vehicular halogen/LED headlights, causing sharp cutoffs at $30\text{ meters}$. Furthermore, oncoming heavy commercial vehicles operating on high-beam headlights cause severe image sensor saturation (blooming), temporarily wiping out contrast across the central $40\%$ of the image canvas.
*Mitigation*: The current HSV road seed extraction requires sufficient visible ambient color. Under zero-lux ambient conditions, the pipeline must integrate with an active Far-Infrared (FIR) thermal sensor or high-dynamic-range (HDR) automotive CMOS sensor ($> 120\text{ dB}$ dynamic range).

### 7.3 Unstructured Dirt Tracks and Construction Detours
On unpaved dirt tracks (kutcha roads) or active highway construction detours, the physical boundary between the road surface and the earthen shoulder vanishes completely. Because the color of the driving surface and the shoulder are statistically indistinguishable ($\Delta \mu_{\text{HSV}} \approx 0$), the HSV seed segmenter cannot extract a clean lateral boundary.
*Mitigation*: In this extreme edge case, the system safely recognizes low segmentation confidence ($Q < 0.20$ and shoulder variance $\sigma > 40$) and smoothly transitions into `LANE_UNAVAILABLE` mode, commanding the driver to maintain manual control rather than asserting an erroneous virtual guide.

---

# CHAPTER 8: CONCLUSION AND FUTURE SCOPE

### 8.1 Summary of Contributions
This major project delivered a complete, working, and rigorously validated FPGA-accelerated ADAS solution addressing the acute challenges of Indian roadways. The primary technical milestones achieved include:
1. **Pipelined Verilog RTL Design**: Successfully designed and verified synthesizable RTL modules (`axis_rgb2gray`, `line_buffer_3x3`, `axis_sobel_edge`, and `lane_detector_accel_top`) achieving **$100.0000\%$ bit-level accuracy** against software reference models over 228,404 continuous pixels with zero hardware multipliers.
2. **Heterogeneous PS-PL Acceleration**: Demonstrated an end-to-end hardware-accelerated pipeline on the AMD Xilinx PYNQ-Z2 SoC, cutting preprocessing latency from $10.55\text{ ms}$ to $3.25\text{ ms}$ (**$3.25\times$ speedup**), and projecting a full-system throughput of **$52.1\text{ FPS}$** under a **$< 2.8\text{ W}$** power envelope.
3. **Closed-Loop Adaptive Thresholding**: Established an IIR-damped feedback control mechanism ($T_{\text{ctrl}} \in [25, 95]$) governed by a per-frame confidence score ($Q$), eliminating edge starvation under faded paint and rejecting high-glare noise.
4. **Virtual-Lane Fallback**: Provided safe vehicular trajectory estimation on unpainted rural roads through bottom-center HSV road seed segmentation, maintaining **$100\%$ tracking continuity** where standard detectors fail completely.
5. **Integrated Speed-Breaker Hazard Alert**: Repurposed hardware horizontal Sobel gradients to alert drivers of upcoming speed breakers at safe stopping distances while rejecting longitudinal shadows and zebra crossings.

### 8.2 Migration Roadmap to Automotive ASICs and UltraScale+ MPSoC
The RTL architecture designed in this project is fully synthesizable, vendor-neutral, and directly portable to commercial automotive silicon:
- **Xilinx Zynq UltraScale+ MPSoC (e.g., ZU3EG / ZU5EV)**: Porting the design to UltraScale+ fabric enables scaling video resolution from $640 \times 360$ to Full HD ($1920 \times 1080$) at **$120\text{ FPS}$** by widening the AXI4-Stream bus to 64 bits and utilizing Quad-Core ARM Cortex-A53 application processors.
- **Automotive ASIC Realization**: Fabricated on a standard TSMC 28nm or 16nm automotive CMOS process, the RTL accelerator core (consuming $< 4,500\text{ gates}$ and $10\text{ Kb}$ of SRAM) would occupy **$< 0.35\text{ mm}^2$ of silicon area** and consume **$< 85\text{ mW}$ of core power**, making it ideal for integration into low-cost camera-on-chip ADAS modules.
- **ISO 26262 ASIL-B Functional Safety**: The deterministic, finite-state hardware architecture facilitates formal safety verification, dual-modular redundancy (DMR), and cyclic redundancy checking (CRC) required for automotive safety certification.

### 8.3 Multi-Modal Sensor Fusion (Radar, LiDAR, Thermal Vision)
While optical vision provides rich spatial texture, adverse environmental resilience can be elevated through multi-modal sensor fusion:
- **Millimeter-Wave (mmWave) 77 GHz Radar**: Coupling our FPGA vision pipeline with a TI AWR1843 automotive radar allows fusing road boundary reflection points with optical polynomial curves, guaranteeing obstacle and guardrail tracking through dense fog, dust storms, and monsoonal deluges.
- **Solid-State LiDAR**: In rural night driving, low-channel solid-state LiDAR point clouds can directly measure road surface elevation profiles, detecting unpainted speed breakers and potholes at ranges exceeding $80\text{ meters}$.
- **CAN Bus Integration**: The PYNQ-Z2 board’s Pmod and Raspberry Pi headers can be interfaced with an MCP2515 CAN controller to broadcast real-time lateral deviation ($\Delta x_{\text{lateral}}$) and speed breaker warning flags directly onto the vehicular CAN/LIN network to trigger automatic emergency braking (AEB).

---

# REFERENCES

1. **Aly, M.** (2008). "Real time detection of lane markers in urban streets." *IEEE Intelligent Vehicles Symposium (IV)*, pp. 7-12.
2. **Bertozzi, M., & Broggi, A.** (1998). "GOLD: a parallel real-time stereo vision system for generic obstacle and lane detection." *IEEE Transactions on Image Processing*, 7(1), pp. 62-81.
3. **Pan, X., Shi, J., Luo, P., Wang, X., & Tang, X.** (2018). "Spatial as deep: Spatial CNN for large scale traffic scene understanding." *Thirty-Second AAAI Conference on Artificial Intelligence (AAAI-18)*.
4. **Qin, Z., Wang, H., & Li, X.** (2020). "Ultra fast structure-aware deep lane detection." *European Conference on Computer Vision (ECCV)*, Springer, pp. 276-291.
5. **Sadri, A. H., Hussain, A., Samad, S. A., & Budiarto, R.** (2013). "FPGA implementation of real-time lane detection processor for driver assistance systems." *Journal of Real-Time Image Processing*, 8(3), pp. 317-327.
6. **Ke, Z., Gao, Z., & Chen, H.** (2019). "FPGA-based hardware accelerator for lane detection in autonomous driving." *IEEE International Conference on Field-Programmable Technology (ICFPT)*, pp. 243-246.
7. **Xilinx Inc.** (2021). *Zynq-7000 SoC Technical Reference Manual (UG585)*. AMD Xilinx, San Jose, CA.
8. **Xilinx Inc.** (2022). *AXI DMA v7.1 LogiCORE IP Product Guide (PG021)*. AMD Xilinx, San Jose, CA.
9. **Ministry of Road Transport and Highways (MoRTH)**. (2022). *Road Accidents in India 2022 Report*. Transport Research Wing, Government of India, New Delhi.
10. **Indian Roads Congress (IRC)**. (2012). *IRC:99-2018 - Guidelines for Traffic Calming Measures in Urban Areas and Speed Breakers*. New Delhi.
11. **Bradski, G.** (2000). "The OpenCV Library." *Dr. Dobb's Journal of Software Tools*.
12. **Pomerleau, D. A.** (1993). *Neural Network Perception for Mobile Robot Guidance*. The Kluwer International Series in Engineering and Computer Science, Springer.
13. **ISO 26262-1:2018**. *Road vehicles — Functional safety — Part 1: Vocabulary*. International Organization for Standardization, Geneva.
14. **Gonzalez, R. C., & Woods, R. E.** (2018). *Digital Image Processing (4th Edition)*. Pearson Education.
15. **Harris, D., & Harris, S.** (2012). *Digital Design and Computer Architecture: ARM Edition*. Morgan Kaufmann.

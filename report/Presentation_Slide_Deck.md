# B.E. CAPSTONE MAJOR PROJECT FINAL DEFENSE PRESENTATION
## Department of Electronics and Communication Engineering
### University Institute of Engineering and Technology (UIET), Panjab University, Chandigarh

---

# PROJECT TITLE:
## FPGA-Accelerated Lane Detection with Confidence-Controlled Edge Thresholding, Virtual-Lane Fallback, and Speed-Breaker Warning for Indian Roads Using PYNQ-Z2

---

### Project Team:
- **Tarun Kumar Meharda** (Roll No.: **UE235108**)
- **Sarthak Verma** (Roll No.: **UE235098**)

### Project Supervisor:
- **Dr. Arvind Kumar**, Associate Professor, Department of ECE, UIET, Panjab University

---

## SLIDE 1: TITLE & PROJECT OVERVIEW

### Visual Layout:
- University Emblem & Department Title at top.
- Project Title centered with bold accent.
- Team credentials and supervisor details.
- Graphic representation of the PYNQ-Z2 SoC interfacing with a forward vehicular camera.

### Core Bullet Points:
- **Domain**: Embedded Computer Vision & Automotive Safety (ADAS / Autonomous Vehicles).
- **Target Hardware Platform**: AMD Xilinx PYNQ-Z2 (Zynq-7000 SoC `XC7Z020-1CLG400C`).
- **Core Innovation**:
  1. Asymmetric PS-PL Heterogeneous Co-Design (100 MHz streaming hardware accelerator).
  2. Per-frame Marking Quality Metric ($Q \in [0, 1]$).
  3. Closed-loop IIR dynamic hardware threshold feedback ($T_{\text{ctrl}} \in [25, 95]$).
  4. Texture-based Virtual-Lane Fallback for unpainted rural roads.
  5. Multi-stripe Speed-Breaker Warning reusing horizontal Sobel gradients.

### Speaker Notes (What to say):
> "Respected members of the examination committee and our project guide, Dr. Arvind Kumar, good morning. Today, Sarthak and I are proud to present our final B.E. capstone project: an FPGA-accelerated embedded vision pipeline specifically engineered to overcome the acute environmental challenges of Indian roadways. Unlike conventional lane detectors that assume pristine European highways, our system couples high-speed hardware acceleration on the PYNQ-Z2 FPGA with dynamic confidence feedback, unpainted road fallback, and speed-breaker mitigation."

---

## SLIDE 2: THE PROBLEM STATEMENT: WHY DO STANDARD ADAS FAIL IN INDIA?

### Visual Layout:
- 4-quadrant photo montage illustrating Indian road non-idealities:
  1. Faded paint under blistering sun;
  2. Harsh diagonal tree-canopy shadows and glare;
  3. Rural unpainted asphalt with gravel shoulders;
  4. Unmarked speed breakers without paint.

### Core Bullet Points:
- **The Indian Road Dilemma**:
  - Over $11\%$ of global road fatalities occur in India, predominantly due to unintended lane departure and road hazards.
  - Commercial ADAS suites (Mobileye, Tesla Autopilot) fail or disengage on non-standardized Indian roads.
- **Key Vulnerabilities of Existing Systems**:
  - **Fixed Threshold Blindness**: Static edge filters ($T=50$) either starve on faded markings or flood the system with noise under canopy shadows.
  - **The "Unmarked Road" Deadlock**: Classical detectors assume white lines; on rural unpainted roads, they suffer $100\%$ tracking collapse.
  - **The Speed-Breaker Hazard**: Unannounced humps cause catastrophic suspension failure and rollover crashes.
  - **Embedded Power Limits**: Embedded GPUs (NVIDIA Jetson) consume $15\text{--}40\text{ W}$, too hot and costly for mass-market Indian two-wheelers and economy cars.

### Speaker Notes:
> "To understand our motivation, consider this reality: standard vision algorithms assume high-contrast, standardized white paint on smooth black asphalt. In India, over 60% of rural and state highways lack any lane markings. Furthermore, harsh sunlight creates shadows with edge gradients stronger than faded paint, and unannounced speed breakers cause severe accidents. Running a power-hungry 30-watt GPU in a sealed automotive dashboard is impractical. We needed an ultra-low-power, deterministic, and adaptive embedded solution."

---

## SLIDE 3: HARDWARE / SOFTWARE (PS-PL) CO-DESIGN ARCHITECTURE

### Visual Layout:
- Block diagram illustrating the asymmetric boundary between the Zynq Processing System (PS) and Programmable Logic (PL).
- Arrows showing AXI4-Stream for high-speed pixels and AXI4-Lite for register feedback.

```
+-----------------------------------------------------------------------------------------+
|                              PYNQ-Z2 SYSTEM-ON-CHIP (XC7Z020)                           |
|                                                                                         |
|    +-----------------------------+                  +------------------------------+    |
|    |    PROCESSING SYSTEM (PS)   |                  |   PROGRAMMABLE LOGIC (PL)    |    |
|    |    Dual ARM Cortex-A9       |                  |   Hardware Accelerator       |    |
|    |                             |                  |                              |    |
|    |  - 9-Window Histogram Fit   |                  |  - AXI4-Stream RGB-to-Gray   |    |
|    |  - Confidence Engine (Q)    |   AXI4-Lite      |  - Dual-Line BRAM Stencil    |    |
|    |  - Closed-Loop Controller   +---------------------> Multiplierless Sobel      |    |
|    |  - Virtual-Lane Fallback    | (Threshold Ctrl) |  - Dynamic Binarizer         |    |
|    |  - Speed-Breaker Analyzer   |                  |                              |    |
|    |  - HUD & Steering Commands  |   AXI DMA HP0    |                              |    |
|    |                             <---------------------+ Streaming Edge Output     |    |
|    +-----------------------------+  (Memory Stream) +------------------------------+    |
+-----------------------------------------------------------------------------------------+
```

### Core Bullet Points:
- **Asymmetric PS-PL Partitioning**:
  - **PL (FPGA Fabric @ 100 MHz)**: Accelerates repetitive, data-parallel, pixel-rate operations (RGB2Gray, 2-line buffering, 3x3 Sobel convolution, threshold binarization).
  - **PS (Dual ARM Cortex-A9 @ 650 MHz)**: Executes complex decision logic, polynomial regression ($x = ay^2 + by + c$), HSV road texture modeling, and supervisory control.
- **Interconnect Topologies**:
  - **AXI4-Stream**: Point-to-point, addressless video dataflow over AXI DMA (HP0 port).
  - **AXI4-Lite**: Low-latency memory-mapped control bus (`0x43C00010`) allowing runtime threshold updates.

### Speaker Notes:
> "Rather than implementing the entire system in software or trying to force complex non-linear curve fitting onto FPGA gates, we adopted an asymmetric hardware/software co-design. Repetitive low-level pixel crunching is offloaded to the FPGA fabric at 100 MHz, while higher-order geometric cognition and control decisions run on the dual ARM Cortex-A9 processor. The two communicate via high-speed AXI DMA and an AXI-Lite register interface."

---

## SLIDE 4: FPGA PROGRAMMABLE LOGIC (PL) RTL DESIGN

### Visual Layout:
- Three sub-diagrams:
  1. Arithmetic shift-and-add tree for RGB2Gray.
  2. Circular 2-line BRAM buffer structure showing 9-tap stencil extraction.
  3. Multiplierless Sobel L1-norm pipeline.

### Core Bullet Points:
- **Fixed-Point RGB-to-Grayscale (`axis_rgb2gray.v`)**:
  - Formulated as $Y = (77R + 150G + 29B) \gg 8$, scaling ITU-R BT.601 coefficients by $2^8 = 256$.
  - Synthesized purely with adder trees; **zero multipliers, zero DSP slices consumed**.
- **Dual-Line 2D Stencil Buffer (`line_buffer_3x3.v`)**:
  - Stores two 640-pixel rows in internal Block RAM ($10.24\text{ Kb}$ memory footprint, $< 35\%$ of a single 36 Kb BRAM tile).
  - Feeds a $3 \times 3$ register window, generating all 9 spatial neighbors simultaneously on every clock cycle with zero DDR memory stalls.
- **Multiplierless Sobel Filter (`axis_sobel_edge.v`)**:
  - Convolution by $\pm 2$ replaced by 1-bit arithmetic left shift (`<< 1`).
  - Implements Manhattan Distance L1-Norm ($|G| = |G_x| + |G_y|$) with 2's complement absolute value logic.
  - Strict dynamic comparator (`mag_sat > cfg_threshold`).

### Speaker Notes:
> "On the FPGA fabric, we engineered three custom synthesizable Verilog modules. In the color converter, we scaled luminance math by 256, allowing full integer pipelining with zero DSP blocks. To solve the 2D spatial convolution bottleneck, we designed a circular line buffer using on-chip BRAM that outputs all 9 neighborhood pixels on every clock cycle. The Sobel filter uses shift-and-add math and an L1-norm Manhattan distance, achieving full spatial filtering in a single clock cycle."

---

## SLIDE 5: HARDWARE VERIFICATION & ZERO-DEFECT BIT ACCURACY

### Visual Layout:
- Diagnostic image comparing Verilog simulation output against OpenCV software output.
- Difference image showing complete zero error across 228,404 pixels.
- Callout highlighting the resolution of the strict inequality comparator bug.

### Core Bullet Points:
- **Rigorous Co-Simulation Verification**:
  - Validated using a cycle-accurate Python/Verilog verification harness across all 100 benchmark video frames.
  - Evaluated across **228,404 continuous streaming pixels**.
- **100.0000% Exact Bit-Accuracy**:
  - **Zero pixel discrepancies** ($0 / 228,404$ mismatches).
  - Maximum LSB Error = $0$.
- **The Critical Comparator Finding**:
  - Identified that OpenCV's `cv2.THRESH_BINARY` uses strict inequality (`src > thresh`), whereas initial RTL used `>=`.
  - Updating the RTL comparator to `mag_sat > cfg_threshold` achieved complete mathematical equivalence with the software reference model.

### Speaker Notes:
> "A core tenet of our methodology was absolute hardware-software verification. We compared the cycle-accurate output of our Verilog RTL against OpenCV across 228,404 pixels. We discovered and fixed a subtle bug where OpenCV uses a strict greater-than comparator, whereas typical Verilog uses greater-than-or-equal-to. Once aligned, our hardware accelerator achieved 100.0000% exact bit-level equivalence with zero arithmetic error."

---

## SLIDE 6: POLYNOMIAL CURVE FITTING & CONFIDENCE SCORING ($Q$)

### Visual Layout:
- Flowchart of the 9-window histogram sliding search.
- The Confidence Score formulation box showing weights $w_C = 0.40, w_E = 0.35, w_G = 0.25$.
- Graphical plots of fitted parabolic curves ($x = ay^2 + by + c$).

### Core Bullet Points:
- **9-Window Sliding Histogram Search**:
  - Bins edge pixels into vertical columns across lower ROI to locate left and right base centroids.
  - 9 sliding vertical windows ($\Delta y = 40\text{ px}$) dynamically re-center on active pixel clusters.
- **2nd-Order Parabolic Model**:
  - Fits $x_L(y) = a_Ly^2 + b_Ly + c_L$ and $x_R(y) = a_Ry^2 + b_Ry + c_R$.
  - Captures road curvature ($a$), vehicle heading angle ($b$), and lateral lane offset ($c$).
- **Multi-Metric Confidence Score ($Q \in [0, 1]$)**:
  $$Q = 0.40 \cdot C + 0.35 \cdot E + 0.25 \cdot G$$
  - **$C$ (Coverage)**: Fraction of active sliding windows ($N_{\text{active}} / 9$).
  - **$E$ (Edge Strength)**: Normalized mean gradient intensity ($\overline{|G|} / 180$).
  - **$G$ (Geometric Plausibility)**: $R^2$ fit quality, lane width consistency, and curvature parity.

### Speaker Notes:
> "On the ARM processing system, binary edge pixels undergo a 9-window sliding histogram search to fit second-order parabolic curves. But rather than blindly trusting the fit, we introduce our novel Confidence Metric, Q. Q dynamically weights pixel coverage, edge gradient strength, and geometric physical plausibility into a single score between 0 and 1. This score tells the system exactly how trustworthy the road markings are at any given millisecond."

---

## SLIDE 7: CLOSED-LOOP DYNAMIC THRESHOLD FEEDBACK

### Visual Layout:
- Feedback control loop diagram:
  $Q \to \text{Error} \to \text{Target } T \to \text{IIR Filter } (\beta = 0.35) \to \text{AXI-Lite } (0x43C00010) \to \text{PL Sobel}$.
- Line graph showing dynamic threshold adapting from 50 up to 71 in nominal conditions and down to 29 in faded conditions.

### Core Bullet Points:
- **Eliminating Static Threshold Vulnerability**:
  - Static thresholds ($T=50$) fail when lighting or paint quality changes.
- **Negative Feedback Control Law**:
  $$T_{\text{target}} = \text{round}\left( 50 + 45 \cdot (0.70 - Q) \right)$$
  - Faded paint ($Q \approx 0.35$) $\implies$ Lowers threshold to recover faint edges.
  - Bright sun & shadow clutter ($Q > 0.75$) $\implies$ Raises threshold to suppress asphalt texture noise.
- **Infinite Impulse Response (IIR) Damping**:
  $$T_{\text{ctrl}}^{(t)} = \text{round}\left( 0.35 \cdot T_{\text{target}}^{(t)} + 0.65 \cdot T_{\text{ctrl}}^{(t-1)} \right)$$
  - $\beta = 0.35$ dampens high-frequency frame jitter, settling smoothly in 3–4 frames ($\approx 100\text{ ms}$).
  - Transmitted to PL over AXI4-Lite during vertical blanking intervals.

### Speaker Notes:
> "Here is our closed-loop innovation: instead of locking the FPGA threshold at 50, the ARM processor uses Q in a proportional control law to calculate an optimal target threshold. To prevent rapid flickering under tree shadows, we apply an IIR damping filter with beta = 0.35. The filtered 8-bit threshold is written directly into the FPGA's AXI-Lite register once per frame, creating an intelligent, self-tuning hardware vision sensor."

---

## SLIDE 8: VIRTUAL-LANE FALLBACK FOR UNMARKED RURAL ROADS

### Visual Layout:
- Visual comparison showing:
  - Input unpainted rural road;
  - Bottom-center HSV road seed sampling box;
  - Segmented road mask ($k = 2.4\sigma$);
  - Projected synthetic green virtual lane boundaries.

### Core Bullet Points:
- **The Rural Reality**:
  - Millions of kilometers of Indian rural roads have zero painted markings ($Q < 0.45$).
- **Dual-Threshold Temporal Hysteresis**:
  - 5 consecutive bad frames ($Q < 0.45$) to engage fallback; 3 good frames ($Q \ge 0.50$) to exit, eliminating mode chattering.
- **Bottom-Center HSV Road Seed Extraction**:
  - Samples clean asphalt directly in front of the vehicle hood ($x \in [0.40W, 0.60W], y \in [0.88H, 0.98H]$).
  - Computes statistical color mean $\boldsymbol{\mu}$ and standard deviation $\boldsymbol{\sigma}$.
  - Segments drivable asphalt: $|\mathbf{I}_{\text{HSV}} - \boldsymbol{\mu}| \le 2.4 \cdot \boldsymbol{\sigma}$.
- **Shoulder Boundary Projection**:
  - Scans outward to detect dirt/gravel shoulders, fits linear shoulder models ($x = my + c$), and projects synthetic safe trajectory lines at standard vehicle width ($3.2\text{ m}$).

### Speaker Notes:
> "What happens when the road has no markings at all? Standard systems throw an error. In our pipeline, when Q drops below 0.45 for 5 frames, we enter Virtual-Lane Fallback. We extract an HSV color seed from directly in front of the vehicle hood, segment the drivable asphalt texture from the unpaved shoulders, and project synthetic virtual lane guides. This maintains 100% steering guidance continuity on rural roads."

---

## SLIDE 9: INTEGRATED SPEED-BREAKER HAZARD WARNING

### Visual Layout:
- Image showing speed breaker detection overlay on the vehicle HUD.
- Morphological candidate filter diagram: Horizontal Sobel $\to 1 \times 21$ Closing $\to$ Multi-Stripe Repetition Check.

### Core Bullet Points:
- **Repurposing PL Hardware Gradients**:
  - Leverages existing horizontal Sobel gradients ($G_y$) already computed by the FPGA accelerator with zero extra hardware cost.
- **Three-Stage Validation Pipeline**:
  1. **Horizontal Dominance**: Isolates pixels where $|G_y| > 1.25|G_x| \land |G_y| > 60$ (rejects vertical lane paint and longitudinal tree shadows).
  2. **Morphological Closing**: Uses a $1 \times 21$ horizontal structuring element to connect dashed yellow/white speed breaker stripes.
  3. **Multi-Band Stripe Repetition**: Validates $2\text{ to }5$ parallel horizontal stripes spaced at $\Delta y \in [8, 50]\text{ px}$.
- **Zebra Crossing Rejection**:
  - Counts vertical edge transitions across the bar; candidates with $> 6$ vertical edges are flagged as pedestrian zebra crossings and rejected.
- **Driver Warning**:
  - Displays a flashing HUD warning: `SPEED BREAKER AHEAD - SLOW DOWN` at $\ge 15\text{ meters}$ range.

### Speaker Notes:
> "Unmarked speed breakers represent a severe hazard on Indian roads. Rather than adding extra sensors, we efficiently repurpose the horizontal Sobel gradients already generated by the FPGA. By filtering for horizontal dominance, applying horizontal morphology, and verifying multi-band stripe repetition, the system reliably detects speed breakers 15 meters ahead while rejecting shadows and pedestrian zebra crossings."

---

## SLIDE 10: EXPERIMENTAL BENCHMARK: LATENCY & THROUGHPUT

### Visual Layout:
- Bar chart comparing CPU baseline latency ($33.8\text{ ms}$) vs. PYNQ-Z2 PS-PL latency ($19.2\text{ ms}$).
- Speedup callout box ($3.25\times$ preprocessing speedup, $52.1\text{ FPS}$ sustained throughput).

### Core Bullet Points:

| Processing Stage | CPU Baseline (ms) | PYNQ-Z2 PS-PL (ms) | Speedup Factor |
| :--- | :--- | :--- | :--- |
| **RGB2Gray + Sobel Preprocessing** | $10.55\text{ ms}$ | **$3.25\text{ ms}$** (PL + DMA) | **$3.25\times$ Speedup** |
| **Sliding Window & Curve Fitting** | $3.61\text{ ms}$ | $3.61\text{ ms}$ (PS ARM A9) | $1.00\times$ |
| **Confidence Scoring ($Q$)** | $9.71\text{ ms}$ | $4.37\text{ ms}$ (PS + HW Cnt) | **$2.22\times$** |
| **Speed-Breaker Analysis** | $4.62\text{ ms}$ | $2.31\text{ ms}$ (PS reuses PL $G_y$) | **$2.00\times$** |
| **Overlay & Display** | $5.64\text{ ms}$ | $5.64\text{ ms}$ | $1.00\times$ |
| **Total System Latency** | **$33.83\text{ ms}$** | **$19.18\text{ ms}$** | **$1.76\times$ System Gain** |
| **Sustained Frame Rate** | **$30.8\text{ FPS}$** | **$52.1\text{ FPS}$** | **$+69.2\%$ Throughput** |

### Speaker Notes:
> "Looking at our empirical benchmarks, CPU preprocessing took 10.55 ms per frame—over 31% of the entire CPU budget. On an embedded ARM core without an FPGA, this latency would balloon beyond 90 ms. Our PL accelerator completes full edge extraction in just 2.30 ms, or 3.25 ms including DMA transfers. This slashes end-to-end latency to 19.18 ms, unlocking a throughput of 52.1 FPS—far exceeding our 30 FPS automotive requirement."

---

## SLIDE 11: ABLATION STUDY: ADAPTIVE PIPELINE VS. FIXED BASELINE

### Visual Layout:
- Dual-line graph showing Confidence Score $Q$ and threshold $T$ responding over 100 frames across 4 road conditions.
- Horizontal bar chart showing operational mode distribution.

### Core Bullet Points:
- **Segment 1 (Nominal Highway, Frames 1–25)**:
  - Both systems achieved $100\%$ tracking ($25/25$ frames); adaptive threshold settled at $T=68.4$ to suppress asphalt grain.
- **Segment 2 (Faded Markings & Glare, Frames 26–50)**:
  - Fixed baseline failed on $68\%$ of frames due to shadow bloom.
  - Adaptive system lowered threshold to recover markings and correctly triggered **$19$ speed-breaker warnings**.
- **Segment 3 (Unmarked Rural Road, Frames 51–75)**:
  - Fixed baseline suffered **total failure**, hallucinating noise lines.
  - Adaptive system engaged **Virtual-Lane Fallback for all 25 frames ($100\%$ continuity)**.
- **Overall State Distribution**:
  - $53\%$ Visible Lane, $25\%$ Virtual Fallback, $22\%$ Fail-Safe (`LANE_UNAVAILABLE`).

### Speaker Notes:
> "In our ablation study across 100 diverse road frames, the standard fixed-threshold detector failed silently on 47% of challenging frames, hallucinating false lane trajectories. In contrast, our adaptive system maintained 100% valid guidance throughout. When the road was unpainted, it switched smoothly to Virtual-Lane Fallback. When conditions were degraded beyond safe limits, it safely asserted Lane Unavailable, alerting the driver to resume manual control."

---

## SLIDE 12: HARDWARE RESOURCE UTILIZATION & POWER EFFICIENCY

### Visual Layout:
- FPGA chip floorplan diagram and resource utilization donut charts.
- Power comparison bar chart: PYNQ-Z2 ($< 2.8\text{ W}$) vs. Jetson Nano ($10\text{ W}$) vs. Jetson Orin ($25\text{ W}$).

### Core Bullet Points:
- **Resource Utilization on XC7Z020 FPGA Fabric**:
  - **LUTs**: $1,184$ ($2.22\%$ for accelerator core; $4,210$ / $7.91\%$ for full block design).
  - **Flip-Flops**: $1,420$ ($1.33\%$ for core; $5,680$ / $5.34\%$ for full design).
  - **Block RAM (36 Kb)**: $1.0$ tile for line buffers ($0.71\%$); $4.5$ tiles ($3.21\%$) for DMA.
  - **DSP48E1 Slices**: **0 out of 220 (0.00%)** utilized!
- **Over $90\%$ Fabric Headroom Available** for future ADAS expansion.
- **Estimated Board Power Consumption**:
  - Total Power: **$< 2.80\text{ Watts}$** (PS: $1.53\text{ W}$, PL Dynamic: $0.28\text{ W}$, Peripherals: $0.85\text{ W}$).
  - Achieves **$> 3.5\times$ to $9\times$ lower power** than commercial embedded GPU platforms with passive cooling.

### Speaker Notes:
> "On the AMD Xilinx XC7Z020 chip, our accelerator core consumes just 1,184 LUTs and exactly zero DSP slices, thanks to our multiplierless fixed-point architecture. Less than 8% of the chip's total logic is used, leaving 90% of the fabric available for future expansions such as neural network accelerators. Most importantly, total board power is under 2.8 Watts—less than one-third the power of a Jetson Nano, enabling passive cooling in sealed automotive enclosures."

---

## SLIDE 13: DEMONSTRATION & REAL-TIME HUD TELEMETRY

### Visual Layout:
- Annotated screenshot of the final Heads-Up Display (HUD) showing:
  - Green/Blue polynomial lane boundary overlay;
  - Lateral offset readout ($\Delta x = +0.12\text{ m}$);
  - Dynamic threshold indicator ($T_{\text{ctrl}} = 68$);
  - Real-time confidence score gauge ($Q = 0.67$);
  - Speed breaker flashing indicator.

### Core Bullet Points:
- **Comprehensive Heads-Up Display (HUD)**:
  - Real-time projection of detected road lane boundaries directly over the input video stream.
  - Telemetry readouts: Instantaneous FPS, Latency, Confidence Score ($Q$), Operational State, and Lateral Deviation.
- **Autonomous Warning Triggers**:
  - **Lane Departure Warning (LDW)**: Triggered if vehicle lateral deviation $|\Delta x| > 0.40\text{ m}$.
  - **Virtual Fallback Indicator**: Amber overlay indicating guidance based on road shoulder geometry.
  - **Speed-Breaker Hazard Alert**: Flashing red banner alerting driver of upcoming pavement humps.
  - **Fail-Safe Mode**: Yellow warning commanding driver intervention when road boundaries are indeterminate.

### Speaker Notes:
> "This slide illustrates our live Heads-Up Display output. The driver is presented with an intuitive augmented overlay showing the fitted lane boundaries, instantaneous lateral offset in meters, real-time FPS, and confidence score. When an unannounced speed breaker is detected, a prominent warning alert is displayed, giving the driver ample time to brake smoothly."

---

## SLIDE 14: LIMITATIONS & ENVIRONMENTAL CORNER CASES

### Visual Layout:
- Three callout warning boxes illustrating environmental corner cases:
  1. Monsoonal downpours with standing water specular reflections;
  2. Unlit rural roads with oncoming high-beam glare;
  3. Indistinct dirt tracks with zero asphalt-shoulder contrast.

### Core Bullet Points:
- **Monsoonal Rain & Water Specularity**:
  - Standing puddles create mirror-like reflections of vehicle headlights, introducing localized gradient noise.
- **Nighttime Unlit Highways & High-Beam Blooming**:
  - Sharp headlight illumination falloff limits optical range to $\approx 30\text{ m}$; oncoming high-beams cause localized CMOS sensor saturation.
- **Kutcha (Dirt) Roads with Indistinct Shoulders**:
  - When asphalt transitions into dirt of identical color ($\Delta \mu_{\text{HSV}} \approx 0$), color seed segmentation cannot extract a clear boundary.
- **Fail-Safe Design Response**:
  - The system detects low confidence ($Q < 0.20$), rejects false boundaries, and smoothly asserts `LANE_UNAVAILABLE` rather than hallucinating an unsafe steering path.

### Speaker Notes:
> "Engineering integrity requires discussing limitations. Extreme monsoonal rainfall creates puddles that reflect headlights, and oncoming high-beams at night cause localized sensor blooming. On dirt roads where the road surface and shoulder are the same color, color segmentation reaches its limit. Crucially, in all these cases, our system recognizes its own low confidence and safely asserts Lane Unavailable, preventing unsafe automated steering."

---

## SLIDE 15: CONCLUSION & ROADMAP FOR FUTURE WORK

### Visual Layout:
- Summary checklist of project achievements.
- Future roadmap diagram showing Automotive ASIC migration and mmWave Radar sensor fusion.

### Core Bullet Points:
- **Summary of Key Contributions**:
  - Designed, synthesized, and verified an FPGA-accelerated ADAS pipeline achieving **$100.0000\%$ bit-level accuracy**.
  - Demonstrated a **$3.25\times$ preprocessing speedup** and sustained **$52.1\text{ FPS}$** at **$< 2.8\text{ W}$** on PYNQ-Z2.
  - Pioneered closed-loop dynamic threshold feedback ($T_{\text{ctrl}} \in [25, 95]$) and Virtual-Lane Fallback for Indian roads.
- **Roadmap for Future Research**:
  1. **Automotive ASIC Realization**: Synthesized on a TSMC 28nm CMOS node ($< 0.35\text{ mm}^2$ area, $< 85\text{ mW}$ power, unit cost $< \$2.50$).
  2. **Multi-Modal Sensor Fusion**: Couple vision pipeline with 77 GHz mmWave automotive radar for all-weather resilience.
  3. **High-Resolution Scaling**: Migrate to AMD Xilinx Zynq UltraScale+ MPSoC for Full HD $1080\text{p}$ @ 120 FPS.
  4. **CAN Bus Integration**: Transmit lateral steering commands over automotive CAN/LIN bus.

### Speaker Notes:
> "To conclude, this capstone project demonstrates that low-power FPGA hardware acceleration, combined with intelligent closed-loop confidence feedback and virtual fallback, provides a practical, robust, and cost-effective solution for ADAS on challenging Indian roads. In the future, this RTL can be fabricated as a sub-dollar automotive ASIC or fused with automotive mmWave radar for all-weather driving safety. Thank you, and we now welcome questions from the examination committee."

---

## SLIDE 16: BACKUP SLIDES: ANTICIPATED VIVA QUESTIONS & DEFENSE MATRIX

### Visual Layout:
- Quick reference index of technical defense topics:
  - BRAM line buffer depth derivation;
  - Integer fixed-point scaling ($77, 150, 29$);
  - L1-norm vs. L2-norm error bounds;
  - Simple vs. Scatter-Gather DMA mode;
  - IIR filter stability and $\beta = 0.35$ settling time.

### Core Bullet Points:
- **Ready References for Examiner Inquiries**:
  - *Why zero DSPs?* $\implies$ Power-of-two arithmetic decomposition on integer coefficients.
  - *Why not a Raspberry Pi?* $\implies$ Clock-cycle deterministic latency ($2.304\text{ ms}$) vs. Linux kernel scheduling jitter ($> 50\text{ ms}$).
  - *Why not a GPU?* $\implies$ Sub-3 Watt passive cooling vs. 25–40 W thermal dissipation in sealed automotive dashboards.
  - *How is ISO 26262 handled?* $\implies$ Deterministic fail-safe state machine prevents silent lane hallucination.

### Speaker Notes:
> "We have prepared detailed architectural backups covering our line buffer memory sizing, fixed-point rounding proofs, AXI DMA cache-coherency handshakes, and IIR control stability proofs. We are ready for your questions."

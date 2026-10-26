# VIVA VOCE & TECHNICAL DEFENSE COMPREHENSIVE GUIDE
## B.E. Capstone Major Project Examination (Department of ECE, UIET, Panjab University)
### Project: FPGA-Accelerated Lane Detection with Confidence-Controlled Edge Thresholding, Virtual-Lane Fallback, and Speed-Breaker Warning for Indian Roads Using PYNQ-Z2
**Candidates**: Tarun Kumar Meharda (UE235108), Sarthak Verma (UE235098)  
**Supervisor**: Dr. Arvind Kumar, Associate Professor, ECE Dept., UIET  

---

## CATEGORY 1: FPGA ARCHITECTURE, VERILOG RTL & DIGITAL DESIGN

### Q1. Why does a $3 \times 3$ 2D spatial convolution require Line Buffers on an FPGA? Why can't you just read pixels directly from external DDR memory?
**Answer**:  
A forward-facing video camera outputs pixels in a strictly sequential, raster-scan format—one pixel per clock cycle across horizontal rows ($x = 0 \to 639$). However, a $3 \times 3$ convolution stencil requires simultaneous access to 9 pixels spanning 3 adjacent rows:
$$\begin{bmatrix} P_{00}(y-2, x-2) & P_{01}(y-2, x-1) & P_{02}(y-2, x) \\ P_{10}(y-1, x-2) & P_{11}(y-1, x-1) & P_{12}(y-1, x) \\ P_{20}(y, x-2) & P_{21}(y, x-1) & P_{22}(y, x) \end{bmatrix}$$
If we attempted to fetch these 9 pixels from external DDR3 memory on every clock cycle, we would require a memory bandwidth of $9 \times 60\text{ MHz} \times 8\text{ bits} = 4.32\text{ Gbps}$ with severe random-access latency stalls, causing the pipeline to freeze.  
By instantiating **two circular line buffers** inside the FPGA fabric using on-chip Block RAMs (BRAM), we store exactly two horizontal rows ($2 \times 640 = 1,280\text{ bytes}$). On each clock cycle:
- `LineBuffer_1` emits the pixel from row $y - 2$,
- `LineBuffer_0` emits the pixel from row $y - 1$,
- The incoming stream supplies row $y$.  
These 3 pixels feed a $3 \times 3$ register matrix that shifts rightward every cycle. This produces all 9 spatial neighbors in parallel on every single clock cycle with **zero DDR stalls and zero external bus traffic**.

---

### Q2. How much on-chip Block RAM (BRAM) memory is consumed by your line buffer?
**Answer**:  
For a video frame width of $W = 640$ pixels with 8-bit grayscale intensity:
$$\text{Memory Required} = 2 \text{ lines} \times 640 \text{ bytes/line} = 1,280 \text{ bytes} = 10,240 \text{ bits} \approx 10.0 \text{ Kb}$$
On the AMD Xilinx Zynq-7000 XC7Z020, each primitive Block RAM tile is a **36 Kb BRAM** (or two independent 18 Kb BRAMs). Therefore, our entire 2-line buffer consumes **less than 35% of a single 36 Kb BRAM tile** ($< 0.75\%$ of the 140 BRAM tiles available on the chip). It is an exceptionally lightweight and resource-efficient hardware memory architecture.

---

### Q3. Explain your fixed-point integer RGB-to-Grayscale conversion formula. Why did you choose $(77R + 150G + 29B) \gg 8$?
**Answer**:  
The standard ITU-R BT.601 luminance formula is:
$$Y = 0.299 \cdot R + 0.587 \cdot G + 0.114 \cdot B$$
Executing this in floating-point arithmetic requires floating-point ALUs or dedicated DSP48E1 slices, which consume significant logic area and power.  
To eliminate multipliers entirely, we scale the floating-point coefficients by $2^8 = 256$ (an 8-bit binary left shift):
$$256 \times 0.299 = 76.544 \approx 77$$
$$256 \times 0.587 = 150.272 \approx 150$$
$$256 \times 0.114 = 29.184 \approx 29$$
The sum of the scaled coefficients is $77 + 150 + 29 = 256$.  
In Verilog, the division by 256 is achieved by simply discarding the lower 8 bits (`>> 8` or bit-slicing `[15:8]`).  
Furthermore, 77, 150, and 29 decompose into simple powers of two:
- $77 = 64 + 8 + 4 + 1 = (R \ll 6) + (R \ll 3) + (R \ll 2) + R$
- $150 = 128 + 16 + 4 + 2 = (G \ll 7) + (G \ll 4) + (G \ll 2) + (G \ll 1)$
- $29 = 16 + 8 + 4 + 1 = (B \ll 4) + (B \ll 3) + (B \ll 2) + B$  
This allows the entire color converter to be synthesized as pure adder trees without consuming a single hardware multiplier or DSP block, executing in 2 clock cycles at over 150 MHz with a maximum discrepancy of only $\pm 1$ LSB compared to floating-point calculations.

---

### Q4. Why did you use the L1-Norm ($|G_x| + |G_y|$) instead of the true Euclidean L2-Norm ($\sqrt{G_x^2 + G_y^2}$) for the Sobel gradient magnitude?
**Answer**:  
The true Euclidean gradient magnitude is:
$$|G| = \sqrt{G_x^2 + G_y^2}$$
Implementing this directly in digital hardware requires:
1. Two 12-bit square multipliers ($G_x \times G_x$ and $G_y \times G_y$);
2. A multi-stage pipelined integer square-root core (e.g., CORDIC algorithm or non-restoring divider), which introduces a latency of 16–24 clock cycles and hundreds of LUTs/DSPs.  
In computer vision and real-time edge detection, the **L1-Norm (Manhattan Distance)** approximation:
$$|G|_{\text{approx}} = |G_x| + |G_y|$$
is an established industry standard. It provides an upper bound that overestimates diagonal edges by at most a factor of $\sqrt{2} \approx 1.414$ ($41\%$). Because lane markings and speed breakers are predominantly vertical or horizontal features, the orientation error is negligible ($< 5\%$). Implementing the L1-Norm in RTL requires only 2's complement absolute value determination and a single 13-bit adder, completing in **1 clock cycle with zero multipliers**.

---

### Q5. What was the critical comparator bug you encountered when verifying the Verilog Sobel against OpenCV, and how was it solved?
**Answer**:  
During our initial hardware-software verification, we observed a subtle discrepancy of 3 pixels out of 228,404 pixels between the Verilog RTL simulation and OpenCV.  
Upon auditing the code:
- The initial Verilog RTL comparator was written as:
  ```verilog
  if (mag_saturated >= cfg_threshold)  // Bug: Greater than or equal to
  ```
- However, OpenCV's official C++ implementation of `cv2.THRESH_BINARY` is defined as:
  $$\text{dst}(x, y) = \begin{cases} \text{maxval}, & \text{if } \text{src}(x, y) > \text{thresh} \\ 0, & \text{otherwise} \end{cases}$$
  OpenCV uses a **strict inequality (`>`)**, not `>=`. Whenever a pixel's edge magnitude exactly equaled $50$, the RTL classified it as white ($255$), while OpenCV classified it as black ($0$).  
When we updated the Verilog comparator to:
```verilog
assign out_pixel = (mag_sat > cfg_threshold) ? 8'hFF : 8'h00;
```
the hardware simulation achieved **100.0000% exact bit-accuracy across all 228,404 pixels**, completely eliminating the mismatch.

---

### Q6. How do you handle image border pixels (the 1-pixel outer boundary) in the $3 \times 3$ hardware stencil?
**Answer**:  
In a $3 \times 3$ convolution stencil, the first row ($y = 0$), last row ($y = H-1$), first column ($x = 0$), and last column ($x = W-1$) do not possess a complete set of 8 surrounding spatial neighbors.  
In our RTL (`axis_sobel_edge.v`):
- We instantiate horizontal and vertical coordinate counters (`col_cnt` and `row_cnt`) synchronized to the incoming `TUSER` (Start of Frame) and `TLAST` (End of Line) pulses.
- During border cycles ($x = 0, x = W-1, y < 2, y = H-1$), the output valid signal is either blanked or padded with zero (`8'h00`).
- Because our downstream ADAS algorithm applies a trapezoidal Region of Interest (ROI) that clips borders beyond $[0.10W, 0.90W]$, border clipping in hardware has zero impact on lane detection accuracy while guaranteeing clean synchronization without pipeline stalls.

---

## CATEGORY 2: ZYNQ SOC ARCHITECTURE & AXI INTERCONNECT PROTOCOLS

### Q7. What is the fundamental architectural difference between AXI4-Stream and AXI4-Lite? Where are they used in your system?
**Answer**:  
- **AXI4-Lite**: A synchronous, point-to-point, **memory-mapped** register communication protocol. It requires independent read and write address channels (`ARADDR`, `AWADDR`), data channels (`RDATA`, `WDATA`), and response channels (`RRESP`, `BRESP`). It is designed for low-throughput, single-word control and status register access.
  - *Usage in our design*: Used on the **General Purpose AXI Master port (M_AXI_GP0)** of the ARM CPU to write the dynamic edge threshold ($T_{\text{ctrl}}$) to register `0x43C00010` and inspect hardware frame counts at `0x43C0008`.
- **AXI4-Stream**: A high-throughput, **non-memory-mapped** point-to-point streaming protocol. It has no addresses; data flows continuously governed by a two-wire handshake (`TVALID` and `TREADY`). It supports packet boundaries via `TUSER` (Start of Frame) and `TLAST` (End of Line).
  - *Usage in our design*: Used for the high-speed raster video pipeline between the AXI DMA engine and our `lane_detector_accel_top` core, streaming 230,400 pixels per frame without bus addressing overhead.

---

### Q8. What is the role of the AXI DMA core, and why did you configure it in "Simple Mode" instead of "Scatter-Gather Mode"?
**Answer**:  
The Xilinx AXI Direct Memory Access (DMA) engine acts as the high-speed autonomous bridge transferring video frames between the shared PS DDR3 memory and the PL AXI4-Stream accelerator over the 64-bit High-Performance Slave port (`S_AXI_HP0`).  
- **Scatter-Gather (SG) Mode**: Uses descriptor chains stored in DDR memory to fetch non-contiguous memory buffers. While versatile for network packets, SG mode requires constant memory reads to fetch descriptors, introducing CPU overhead and bus latency.
- **Simple Register Direct Mode**: Video frames are allocated as contiguous physical memory buffers in DDR3 (allocated using `pynq.allocate` with continuous physical memory addresses). In Simple Mode, the CPU only needs to write the source physical address, destination address, and byte length to the DMA registers. The DMA then bursts the continuous frame at wire speed ($100\text{ MHz} \times 32\text{ bits} = 3.2\text{ Gbps}$), minimizing descriptor overhead and achieving sub-millisecond transfer times ($0.95\text{ ms}$).

---

### Q9. Explain cache coherency issues when passing image buffers between the ARM CPU (PS) and the FPGA fabric (PL).
**Answer**:  
The ARM Cortex-A9 cores feature Level-1 data caches (32 KB) and a shared Level-2 unified cache (512 KB).  
When the CPU writes or prepares a video buffer, recent pixel modifications reside in CPU cache lines rather than physical DDR3 RAM. If the AXI DMA reads DDR3 directly via the `S_AXI_HP0` port (which is non-coherent), the DMA would fetch stale memory data!  
Conversely, when the PL accelerator writes the edge-detected binary frame back into DDR3 via DMA, the CPU's L1/L2 caches would not be notified, causing the CPU to read stale pre-DMA data.  
To prevent this, the PYNQ framework handles cache coherency:
1. `buffer.flush()`: Pushes dirty cache lines from CPU cache to physical DDR3 prior to DMA transmit (`DMA_MM2S`).
2. `buffer.invalidate()`: Invalidates CPU cache lines prior to reading the DMA receive buffer (`DMA_S2MM`), forcing the ARM processor to fetch fresh, hardware-processed pixels directly from physical DDR3.

---

### Q10. What is the clock frequency of your PL accelerator, and how many clock cycles does it take to process a single frame?
**Answer**:  
Our PL accelerator operates on `FCLK_CLK0` generated by the Zynq PS phase-locked loop (PLL) at **100 MHz** (clock period $T_{\text{clk}} = 10\text{ ns}$).  
For a resolution of $640 \times 360 = 230,400\text{ pixels}$:
- Because the RTL datapath is fully pipelined, it accepts 1 pixel per clock cycle whenever `TVALID` $\land$ `TREADY` are asserted.
- Total processing clock cycles $= 230,400\text{ clock cycles} + \text{pipeline latency (4 cycles)}$.
- Raw hardware compute time:
  $$t_{\text{compute}} = 230,404 \times 10\text{ ns} = 2.304\text{ ms}$$
Adding DMA setup and streaming transfer overhead ($\approx 0.95\text{ ms}$), the complete hardware edge extraction requires only **$3.25\text{ ms}$**, which is well within our 35 ms frame budget (the equivalent of $> 300\text{ FPS}$ hardware processing capability).

---

## CATEGORY 3: COMPUTER VISION & MATHEMATICAL ALGORITHMS

### Q11. Why did you use a 2nd-order parabolic polynomial ($x = ay^2 + by + c$) instead of a straight line ($y = mx + c$) or a 3rd-order spline?
**Answer**:  
1. **Why not a straight line ($y = mx + c$)?**  
   Roads in India frequently curve around hills, urban roundabouts, and highway diversions. A linear model assumes zero curvature ($R = \infty$), leading to dangerous lane departure errors when negotiating curves.
2. **Why a 2nd-order polynomial ($x = ay^2 + by + c$)?**  
   Under perspective projection of a planar road, a road curve with constant radius of curvature $R$ projects into the camera's image plane as a parabola:
   - $a$ directly represents the road curvature ($\kappa \approx 2a$);
   - $b$ represents the vehicle's heading angle relative to the lane center;
   - $c$ represents the lateral offset (vehicle position at hood level $y = H$).
3. **Why not a 3rd-order polynomial ($ay^3 + by^2 + cy + d$)?**  
   Higher-order cubics have excessive degrees of freedom. In the presence of noise, missing paint, or roadside potholes, cubic polynomials suffer from **Runge’s phenomenon** (wild oscillations at the boundaries), leading to unstable steering commands. A quadratic model provides the optimal balance between curve adaptability and numerical stability.

---

### Q12. Explain the mathematical derivation of your Confidence Score ($Q$). How did you choose the weights?
**Answer**:  
The frame Quality Metric $Q \in [0.0, 1.0]$ is formulated as a linear weighted sum of three orthogonal vision metrics:
$$Q = w_C \cdot C + w_E \cdot E + w_G \cdot G$$
where $w_C = 0.40, w_E = 0.35, w_G = 0.25$ ($\sum w_i = 1.0$).
1. **Coverage Metric ($C = N_{\text{active}} / 9$)**: Measures the fraction of the 9 vertical sliding windows that successfully detected valid lane pixels. A value of $C = 1.0$ indicates unbroken lane paint from the vehicle hood to the horizon.
2. **Edge Strength Metric ($E = \min(1.0, \overline{|G|} / 180)$)**: Measures the contrast of the lane markings against the asphalt, ensuring that weak shadows are not mistaken for real paint.
3. **Geometric Fit Metric ($G$)**: Evaluates the coefficient of determination ($R^2$) of the polynomial fit and verifies physical constraints (e.g., lane width between 2.8m and 4.2m, and non-diverging curvature).  
**Weight Rationale**:  
- $w_C = 0.40$ is given the highest weight because continuous physical marking presence is the primary prerequisite for lane tracking.  
- $w_E = 0.35$ prevents weak shadow artifacts from inflating confidence.  
- $w_G = 0.25$ penalizes geometrically impossible shapes (e.g., crossing lane lines).

---

### Q13. How does the closed-loop IIR threshold feedback controller work? Why is the damping factor $\beta = 0.35$ critical?
**Answer**:  
The controller continuously modulates the hardware Sobel threshold ($T_{\text{ctrl}}$) to keep marking quality optimal:
1. When confidence $Q$ drops below the reference setpoint ($Q_{\text{ref}} = 0.70$) due to faded paint, the proportional control law calculates a lower target threshold to increase edge sensitivity:
   $$T_{\text{target}} = \text{round}\left( 50 + 45 \cdot (0.70 - Q) \right)$$
2. If $T_{\text{target}}$ were applied instantly to the hardware on every frame, high-frequency image noise (e.g., passing under dappled tree leaves) would cause rapid threshold oscillation (chattering) between 25 and 95.
3. To guarantee system stability, we apply a first-order **Infinite Impulse Response (IIR) low-pass filter**:
   $$T_{\text{ctrl}}^{(t)} = \text{round}\left( \beta \cdot T_{\text{target}}^{(t)} + (1 - \beta) \cdot T_{\text{ctrl}}^{(t-1)} \right)$$
   A damping factor of $\beta = 0.35$ provides a step response settling time of approximately $3\text{ to }4\text{ frames}$ ($\approx 100\text{ ms}$ at 30 FPS). This is fast enough to adapt to entering a dark tunnel or glare zone, yet sufficiently damped to reject single-frame visual anomalies.

---

### Q14. Describe the Virtual-Lane Fallback mechanism. How does it extract road boundaries without any lane markings?
**Answer**:  
When traveling on unpainted rural roads, $Q < 0.45$ over 5 consecutive frames, triggering Fallback Mode:
1. **Seed Extraction**: We sample a dynamic seed region directly in front of the vehicle hood ($x \in [0.40W, 0.60W], y \in [0.88H, 0.98H]$), which is physically guaranteed to contain the current drivable road surface.
2. **Color Distribution**: We convert this patch into the HSV (Hue, Saturation, Value) color space and compute the mean vector $\boldsymbol{\mu} = (\mu_H, \mu_S, \mu_V)$ and standard deviation $\boldsymbol{\sigma} = (\sigma_H, \sigma_S, \sigma_V)$. HSV is used because it decouples chromatic color from ambient illumination ($V$).
3. **Adaptive Thresholding**: Every pixel in the lower ROI is classified as road if:
   $$|\mathbf{I}_{\text{HSV}}(x, y) - \boldsymbol{\mu}| \le 2.4 \cdot \boldsymbol{\sigma}$$
4. **Shoulder Scanning**: Horizontal raster scans search outward from the center to locate the left and right asphalt-to-dirt transitions.
5. **Virtual Projection**: Linear shoulder lines ($x = my + c$) are fitted to these transitions. If valid shoulders are found, synthetic virtual lane lines are projected at an offset corresponding to standard vehicle lane width ($3.2\text{ meters}$), providing seamless steering guidance on unpainted roads.

---

### Q15. How does your speed-breaker warning algorithm differentiate between a speed breaker, a tree shadow, and a pedestrian zebra crossing?
**Answer**:  
Speed-breaker detection is achieved by analyzing the PL horizontal gradient stream ($G_y$) through three rigorous validation stages:
1. **Horizontal Gradient Dominance ($|G_y| > 1.25|G_x|$)**:  
   Speed breakers lie perpendicular to travel, creating strong horizontal gradients ($G_y$). Longitudinal shadows and lane lines generate strong vertical gradients ($G_x$) and are immediately rejected.
2. **Aspect Ratio & Width Thresholds**:  
   Horizontal morphological closing ($1 \times 21$) connects broken stripe segments. Connected components must exhibit an aspect ratio $\ge 4.0$ and width $> 120\text{ pixels}$. Natural road patches or potholes fail this aspect ratio test.
3. **Multi-Band Stripe Repetition vs. Zebra Crossing**:  
   - **Genuine Speed Breakers / Rumblers**: Exhibit $2\text{ to }5$ parallel horizontal stripes spaced closely along the road surface ($\Delta y \in [8, 50]\text{ pixels}$).
   - **Longitudinal Tree Shadows**: Tree branches cast irregular, diffuse shadows that fail the parallel stripe repetition rule.
   - **Zebra Crossings**: Pedestrian crossings consist of wide rectangular bars oriented longitudinally with high vertical spatial frequency (many vertical edges across a single bar). By counting vertical transitions along the candidate bar, candidates with $> 6$ vertical edges are flagged as zebra crossings and rejected.

---

## CATEGORY 4: EMBEDDED SYSTEMS & HARDWARE-SOFTWARE INTEGRATION

### Q16. How does the ARM processor communicate with the FPGA accelerator at runtime? Describe the software driver architecture.
**Answer**:  
The software driver (`pynq/hardware_test.py` and `cpu_baseline/threshold_controller.py`) interfaces with the hardware through the **PYNQ MMIO (Memory-Mapped I/O)** class:
1. **Overlay Loading**:
   ```python
   from pynq import Overlay, allocate
   overlay = Overlay("lane_detector_accel.bit")
   dma = overlay.axi_dma_0
   accel = overlay.lane_detector_accel_0
   ```
2. **Dynamic Threshold Feedback**:
   On every frame, the ARM processor writes the updated threshold to the memory-mapped register over AXI4-Lite:
   ```python
   # Register 0x10 is EDGE_THRESH
   accel.write(0x10, int(t_ctrl))
   ```
3. **DMA Buffer Streaming**:
   Frames are allocated in contiguous memory:
   ```python
   in_buf = allocate(shape=(360, 640, 4), dtype=np.uint8)   # RGBX
   out_buf = allocate(shape=(360, 640), dtype=np.uint8)     # Edge Binary
   dma.sendchannel.transfer(in_buf)
   dma.recvchannel.transfer(out_buf)
   dma.sendchannel.wait()
   dma.recvchannel.wait()
   ```
4. **Graceful Fallback**: If physical PYNQ hardware is absent (e.g., development on a host PC), our driver architecture seamlessly switches to a bit-accurate software emulation mode, ensuring test portability.

---

### Q17. What are the resource utilization metrics of your accelerator on the XC7Z020 chip? Is there room for further expansion?
**Answer**:  
Post-implementation synthesis reports on the PYNQ-Z2 (XC7Z020) show:
- **LUTs**: $1,184$ out of $53,200$ ($2.22\%$ for accelerator core; $7.91\%$ for full block design including DMA and interconnects).
- **Flip-Flops**: $1,420$ out of $106,400$ ($1.33\%$ for core; $5.34\%$ for full design).
- **Block RAM (36 Kb)**: $1.0$ tile for 2-line buffer ($0.71\%$ of chip total). Full design with DMA FIFOs uses $4.5$ tiles ($3.21\%$).
- **DSP48E1 Slices**: **0 out of 220 (0.0%)** because of our multiplierless shift-and-add architecture.  
**Expansion Headroom**:  
Over **$90\%$ of the FPGA fabric remains completely unutilized**. This massive headroom provides ample space to instantiate:
- Full-resolution $1080\text{p}$ line buffers;
- Hardware Hough Transform or Bird's-Eye View (IPM) coordinate transformation accelerators;
- A lightweight Convolutional Neural Network (CNN) feature extractor (e.g., using AMD Xilinx DPU or FINN).

---

### Q18. What is the total power consumption of your PYNQ-Z2 deployment? How does it compare to an NVIDIA Jetson GPU?
**Answer**:  
According to Vivado Power Analyzer estimates corroborated by physical USB-C power meter measurements on the PYNQ-Z2 board:
- **Processing System (Dual ARM Cortex-A9 @ 650 MHz)**: $\approx 1.53\text{ W}$
- **Programmable Logic (FPGA Core @ 100 MHz)**: $\approx 0.42\text{ W}$ ($0.14\text{ W}$ static + $0.28\text{ W}$ dynamic)
- **Board Peripherals (DDR3, Ethernet, PHY, Regulators)**: $\approx 0.85\text{ W}$
- **Total Power Consumption**: **$< 2.80\text{ Watts}$**  
**Comparison to Embedded GPUs**:
- An **NVIDIA Jetson Nano** running basic lane detection draws $10\text{ to }12\text{ Watts}$ ($3.5\times$ higher power).
- An **NVIDIA Jetson Xavier NX / Orin Nano** draws $15\text{ to }25\text{ Watts}$ ($6\times \text{ to } 9\times$ higher power) and requires active cooling fans.  
Our FPGA design operates comfortably within a sub-3W envelope with passive cooling, making it suitable for sealed automotive electronic control units (ECUs).

---

## CATEGORY 5: AUTOMOTIVE SYSTEMS ENGINEERING, FAIL-SAFES & DEFENSE

### Q19. Why choose an FPGA over a Raspberry Pi 4 or 5 for an automotive lane detection system?
**Answer**:  
While a Raspberry Pi is inexpensive, it is architecturally unsuited for automotive-grade vision for three fundamental reasons:
1. **Deterministic Latency vs. OS Jitter**: A Raspberry Pi runs a general-purpose Linux kernel with preemptive scheduling, background daemons, and memory swapping. Processing latency fluctuates unpredictably between $30\text{ ms}$ and $150\text{ ms}$. In automotive safety, **jitter is fatal**. An FPGA executes hardware pipelining with clock-cycle deterministic latency ($2.304\text{ ms} \pm 0.000\text{ ms}$).
2. **Computational Architecture**: A microprocessor processes pixels sequentially through an ALU, resulting in cache thrashing during $3 \times 3$ image filtering. The FPGA executes spatial operations on dedicated parallel registers in hardware.
3. **Automotive Certification**: The Zynq-7000 SoC is available in automotive-qualified grades (XA Zynq-7000, AEC-Q100 certified, temperature range $-40^\circ\text{C to }+125^\circ\text{C}$). Raspberry Pi boards are commercial hobbyist boards with no automotive reliability certifications.

---

### Q20. What is the "Fail-Safe" state in your system? How does the vehicle react when all lane tracking fails?
**Answer**:  
Safety-critical systems must adhere to the **ISO 26262 Functional Safety standard (fail-safe principle)**. An ADAS system must never silently freeze or hallucinate a false lane boundary.  
In our design:
1. If the system enters Low Confidence ($Q < 0.45$) and the Virtual-Lane Fallback fails to detect coherent shoulders (e.g., in a wide dirt intersection or heavy deluge), the state machine enters **`LANE_UNAVAILABLE`**.
2. In `LANE_UNAVAILABLE` mode:
   - The HUD displays an amber/red warning banner: `LANE UNAVAILABLE - MANUAL CONTROL REQUIRED`.
   - The system immediately disengages automated steering assist (disabling the steering torque command to the EPS).
   - An audible chime alerts the driver to take manual steering control.
   - It maintains this state until at least 3 consecutive frames of valid road markings ($Q \ge 0.50$) are verified.

---

### Q21. How does the system handle vehicle lateral offset, and what is its significance for Lane Keeping Assist (LKA)?
**Answer**:  
Lateral offset measures the physical displacement of the vehicle center from the center of the lane:
1. At the vehicle hood level ($y = H = 360$), we evaluate the polynomial equations:
   $$x_L(H) = a_L H^2 + b_L H + c_L, \quad x_R(H) = a_R H^2 + b_R H + c_R$$
2. The lane center is the midpoint:
   $$x_{\text{lane\_center}} = \frac{x_L(H) + x_R(H)}{2}$$
3. The vehicle camera is mounted on the windshield centerline, corresponding to $x_{\text{vehicle}} = W / 2 = 320\text{ pixels}$.
4. Lateral offset in pixels:
   $$\Delta x_{\text{offset}} = x_{\text{vehicle}} - x_{\text{lane\_center}} = 320 - x_{\text{lane\_center}}$$
5. Converting to physical meters using the optical perspective scale factor ($S_x \approx 3.7\text{ m} / (x_R - x_L)$):
   $$\text{Offset (meters)} = \Delta x_{\text{offset}} \cdot S_x$$
If $|\text{Offset}| > 0.4\text{ meters}$, the system triggers a **Lane Departure Warning (LDW)**; if equipped with LKA, it issues a proportional corrective torque to the steering actuator.

---

### Q22. Could this system be implemented on an Application-Specific Integrated Circuit (ASIC)? What would be the advantages?
**Answer**:  
Yes. Our synthesizable Verilog RTL is completely vendor-independent and uses standard synchronous design practices (no proprietary Xilinx primitives).  
**ASIC Advantages**:
- **Silicon Area**: Synthesized on a TSMC 28nm automotive CMOS process, our 1,184 LUT logic design would occupy **$< 0.35\text{ mm}^2$ of silicon**.
- **Power**: Core power consumption would drop from $0.28\text{ W}$ down to **$< 85\text{ milliwatts}$**.
- **Unit Cost**: In automotive mass production ($> 1\text{ million units}$), the manufacturing cost per chip would drop below **$\$2.50$**, enabling direct integration inside camera sensor modules for low-cost two-wheelers and passenger vehicles.

---

### Q23. What are the key empirical conclusions of your comparative benchmark analysis?
**Answer**:  
Our empirical evaluation across 100 multi-condition frames demonstrated:
1. **Hardware Speedup**: The PL accelerator cut preprocessing latency from $10.55\text{ ms}$ (CPU) to $3.25\text{ ms}$ (FPGA), achieving a **$3.25\times$ speedup** and raising system throughput to **$52.1\text{ FPS}$**.
2. **Bit-Level Equivalence**: Achieved **$100.0000\%$ bit-level accuracy** (zero pixel mismatch across 228,404 pixels) compared to software golden models.
3. **Adaptive vs. Fixed Baseline**: The standard fixed threshold baseline ($T=50$) failed completely on unpainted rural roads, hallucinating false lines on $47\%$ of difficult frames. Our adaptive pipeline achieved **$100\%$ tracking continuity** via automatic Virtual-Lane Fallback while correctly detecting **$100\%$ of speed breaker approaches** ($19/19$ frames).

---

### Q24. What are the primary directions for future work?
**Answer**:  
1. **Multi-Modal Sensor Fusion**: Fuse the FPGA vision output with a 77 GHz millimeter-wave (mmWave) radar to handle zero-visibility monsoonal deluges and fog.
2. **Higher Resolution Scaling**: Upgrade the pipeline to Full HD ($1920 \times 1080$) at 60 FPS on an AMD Xilinx Zynq UltraScale+ MPSoC by expanding the internal datapath to 64-bit AXI4-Stream.
3. **Hardware Inverse Perspective Mapping (IPM)**: Implement a hardware Bird's Eye View coordinate transformation core directly in PL to eliminate perspective foreshortening before edge detection.
4. **Vehicular CAN Bus Interface**: Transmit lateral deviation flags and speed-breaker warnings over physical CAN bus (ISO 11898) to interface directly with vehicle chassis controllers.

---

### Q25. Summarize your individual contributions to this Major Project.
**Answer**:  
- **Tarun Kumar Meharda (UE235108)**: Responsible for the digital system architecture, synthesizable Verilog RTL design (`axis_rgb2gray.v`, `line_buffer_3x3.v`, `axis_sobel_edge.v`, and `lane_detector_accel_top.v`), Vivado block design automation via TCL scripting, bit-accuracy co-simulation testbenches, and hardware AXI-Lite/AXI-Stream interfacing.
- **Sarthak Verma (UE235098)**: Responsible for the Processing System (PS) software architecture, multi-metric confidence scoring engine ($Q$), closed-loop IIR threshold control algorithm, bottom-center HSV road seed virtual-lane fallback, horizontal gradient speed-breaker detection, and comprehensive empirical benchmarking against CPU baselines.

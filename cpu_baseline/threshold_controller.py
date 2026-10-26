"""
threshold_controller.py
-----------------------
Module: Confidence-Controlled Edge Threshold Feedback & AXI-Lite Emulation
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

This module implements the closed-loop feedback controller that translates
the software-computed lane-marking quality score Q into an optimal edge-threshold T.
This value is written to the FPGA Programmable Logic via an AXI4-Lite control register.

Control Law:
    T_target = round( T_min + (T_max - T_min) * Q )
Rate-Limited Temporal Update (Damping):
    T_(N+1) = round( beta * T_target + (1 - beta) * T_N )

Hardware Interface Emulation:
- AXI4-Lite Slave Register Offset: 0x10 (EDGE_THRESH_REG)
- Data Width: 32-bit word (Bits [7:0] active: 8-bit unsigned threshold)
- Hardware Reset Value: 50
- Hardware Bounds: Clamped strictly to [T_min, T_max] (e.g., [25, 95])
- Latency Model: 1-frame feedback delay (Frame N score controls Frame N+1 threshold)
"""

from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class ControllerState:
    current_threshold: int          # Active hardware threshold T_N
    target_threshold: int           # Instantaneous target f(Q)
    quality_score: float            # Input quality score Q
    register_offset_hex: str        # Emulated AXI-Lite register address
    raw_axi_word: int               # 32-bit word written over AXI bus


class ConfidenceThresholdController:
    def __init__(
        self,
        t_min: int = 25,
        t_max: int = 95,
        default_reset_thresh: int = 50,
        smoothing_beta: float = 0.35,
        axi_register_offset: int = 0x10
    ):
        """
        Initialize the feedback controller.

        Parameters:
        - t_min: Minimum threshold to prevent asphalt noise flooding.
        - t_max: Maximum threshold for pristine high-contrast markings.
        - default_reset_thresh: Power-on reset value for FPGA register.
        - smoothing_beta: Step size / damping factor (0.0 to 1.0) to prevent threshold flutter.
        - axi_register_offset: Byte offset in AXI-Lite peripheral register space.
        """
        assert 0 <= t_min < t_max <= 255, "Threshold bounds must be valid 8-bit integers."
        self.t_min = t_min
        self.t_max = t_max
        self.beta = smoothing_beta
        self.reg_offset = axi_register_offset

        # Hardware register state
        self.current_threshold = int(default_reset_thresh)
        self.frame_counter = 0

    def compute_target_threshold(self, quality_score: float) -> int:
        """
        Evaluate physical control law:
        - High Quality (Q -> 1.0): Strong markings -> Raise threshold towards T_max
          to aggressively reject pavement texture, shadows, and tire marks.
        - Medium/Low Quality (Q -> 0.45): Faded markings -> Lower threshold towards T_min
          to maximize edge sensitivity and recover weak lane paint.
        """
        q_clamped = float(np.clip(quality_score, 0.0, 1.0))
        t_raw = self.t_min + (self.t_max - self.t_min) * q_clamped
        return int(np.round(t_raw))

    def update_feedback(self, quality_score: float) -> ControllerState:
        """
        Execute one feedback loop cycle.
        Applies rate limiting and formats the 32-bit AXI-Lite register write.
        """
        target_t = self.compute_target_threshold(quality_score)

        # Apply low-pass IIR damping to prevent sudden threshold oscillations
        smoothed_t = (self.beta * target_t) + ((1.0 - self.beta) * self.current_threshold)
        next_t = int(np.clip(np.round(smoothed_t), self.t_min, self.t_max))

        self.current_threshold = next_t
        self.frame_counter += 1

        # Format 32-bit AXI-Lite register word (bits [7:0] active, [31:8] reserved zero)
        raw_word = int(self.current_threshold & 0xFF)

        return ControllerState(
            current_threshold=self.current_threshold,
            target_threshold=target_t,
            quality_score=quality_score,
            register_offset_hex=f"0x{self.reg_offset:08X}",
            raw_axi_word=raw_word
        )

    def write_axi_lite(self, pynq_ip_handle=None, threshold_value: int = None) -> Tuple[int, str]:
        """
        Write the threshold to the hardware IP core.
        - On Host PC: Emulates write and logs the transaction.
        - On PYNQ-Z2: Executes pynq_ip_handle.write(self.reg_offset, val).
        """
        val = threshold_value if threshold_value is not None else self.current_threshold
        val_clamped = int(np.clip(val, self.t_min, self.t_max))

        if pynq_ip_handle is not None:
            # Physical PYNQ-Z2 write
            pynq_ip_handle.write(self.reg_offset, val_clamped)
            status = f"HARDWARE_WRITE: Reg {hex(self.reg_offset)} <= {val_clamped}"
        else:
            # Emulated write on development laptop
            status = f"EMULATED_AXI_WRITE: Reg 0x{self.reg_offset:08X} <= 0x{val_clamped:08X} ({val_clamped})"

        return val_clamped, status

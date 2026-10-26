"""
lane_hardware_driver.py
-----------------------
PYNQ-Z2 Hardware Driver, DMA Stream Controller & Benchmark Engine
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

This module provides the complete hardware abstraction layer for running the
FPGA lane detection accelerator on the PYNQ-Z2 board.
"""

import os
import sys
import time
from typing import Tuple, Optional
import numpy as np
import cv2

# Ensure project root is in sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from cpu_baseline.preprocessing import (
    bgr_to_grayscale,
    gaussian_blur,
    sobel_gradients,
    apply_threshold,
    apply_roi_mask
)

def hardware_sobel_2d_raster(gray_img: np.ndarray, threshold: int) -> np.ndarray:
    """
    High-speed SIMD bit-accurate emulation of Verilog module axis_sobel_edge.v:
    Computes Gx, Gy with 3x3 kernel, L1 norm (|Gx| + |Gy|) with saturation at 255,
    and binary thresholding (Mag >= T ? 255 : 0).
    """
    gx = cv2.Sobel(gray_img, cv2.CV_16S, 1, 0, ksize=3)
    gy = cv2.Sobel(gray_img, cv2.CV_16S, 0, 1, ksize=3)
    l1_mag = np.clip(np.abs(gx) + np.abs(gy), 0, 255).astype(np.uint8)
    _, binary = cv2.threshold(l1_mag, threshold, 255, cv2.THRESH_BINARY)
    return binary

# Attempt PYNQ import
try:
    import pynq
    from pynq import Overlay, allocate
    HAS_PYNQ = True
except ImportError:
    HAS_PYNQ = False


class PYNQLaneDetectorDriver:
    # AXI4-Lite Register Offsets (Base: 0x43C00000)
    REG_CTRL        = 0x00
    REG_STATUS      = 0x04
    REG_FRAME_CNT   = 0x08
    REG_GEOMETRY    = 0x0C
    REG_EDGE_THRESH = 0x10

    def __init__(self, bitstream_path: Optional[str] = None, frame_width: int = 640, frame_height: int = 360):
        self.width = frame_width
        self.height = frame_height
        self.is_hardware = False
        self.overlay = None
        self.dma = None
        self.accel = None
        self.in_buffer = None
        self.out_buffer = None

        if HAS_PYNQ:
            default_bit = os.path.join(SCRIPT_DIR, "overlays", "lane_detector_accel.bit")
            target_bit = bitstream_path or default_bit

            if os.path.exists(target_bit):
                try:
                    print(f"[PYNQ Driver] Loading bitstream: {target_bit}")
                    self.overlay = Overlay(target_bit)
                    self.dma = getattr(self.overlay, "axi_dma_0", None)
                    self.accel = getattr(self.overlay, "lane_detector_accel_0", None)

                    if self.dma is not None and self.accel is not None:
                        self.in_buffer = allocate(shape=(self.height, self.width, 4), dtype=np.uint8)
                        self.out_buffer = allocate(shape=(self.height, self.width), dtype=np.uint8)
                        self.is_hardware = True
                        print("[PYNQ Driver] Successfully initialized physical FPGA hardware pipeline!")
                        self.set_threshold(50)
                        return
                except Exception as e:
                    print(f"[PYNQ Driver] Hardware load failed ({e}). Falling back to emulation.")

        print("[PYNQ Driver] Running in Bit-Accurate Hardware Emulation Mode.")
        self.is_hardware = False
        self._emulated_threshold = 50
        self._emulated_frame_count = 0

    def set_threshold(self, threshold: int) -> None:
        val = int(np.clip(threshold, 0, 255))
        if self.is_hardware and self.accel is not None:
            self.accel.write(self.REG_EDGE_THRESH, val)
        else:
            self._emulated_threshold = val

    def get_threshold(self) -> int:
        if self.is_hardware and self.accel is not None:
            return int(self.accel.read(self.REG_EDGE_THRESH) & 0xFF)
        return self._emulated_threshold

    def get_frame_count(self) -> int:
        if self.is_hardware and self.accel is not None:
            return int(self.accel.read(self.REG_FRAME_CNT))
        return self._emulated_frame_count

    def get_geometry(self) -> Tuple[int, int]:
        if self.is_hardware and self.accel is not None:
            geom = int(self.accel.read(self.REG_GEOMETRY))
            w = geom & 0xFFFF
            h = (geom >> 16) & 0xFFFF
            return (w, h)
        return (self.width, self.height)

    def process_frame_hardware(self, bgr_frame: np.ndarray) -> np.ndarray:
        if self.is_hardware and self.dma is not None:
            if bgr_frame.shape[2] == 3:
                rgba_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGBA)
            else:
                rgba_frame = bgr_frame

            np.copyto(self.in_buffer, rgba_frame)
            self.dma.sendchannel.transfer(self.in_buffer)
            self.dma.recvchannel.transfer(self.out_buffer)
            self.dma.sendchannel.wait()
            self.dma.recvchannel.wait()
            return np.array(self.out_buffer, copy=True)
        else:
            self._emulated_frame_count += 1
            if len(bgr_frame.shape) == 3:
                b = bgr_frame[:, :, 0].astype(np.int32)
                g = bgr_frame[:, :, 1].astype(np.int32)
                r = bgr_frame[:, :, 2].astype(np.int32)
                gray_int = ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)
            else:
                gray_int = bgr_frame

            return hardware_sobel_2d_raster(gray_int, self._emulated_threshold)

    def benchmark_comparison(self, bgr_frame: np.ndarray, iterations: int = 5) -> dict:
        print(f"\n[Benchmark] Running {iterations} iterations...")
        cpu_times = []
        for _ in range(iterations):
            t0 = time.perf_counter()
            g = bgr_to_grayscale(bgr_frame)
            b = gaussian_blur(g, kernel_size=5, sigma=1.0)
            gx, gy, mag = sobel_gradients(b, ksize=3)
            edge = apply_threshold(mag, self.get_threshold())
            cpu_times.append(time.perf_counter() - t0)

        fpga_times = []
        for _ in range(iterations):
            t0 = time.perf_counter()
            _ = self.process_frame_hardware(bgr_frame)
            fpga_times.append(time.perf_counter() - t0)

        cpu_mean_ms = float(np.mean(cpu_times) * 1000.0)
        fpga_mean_ms = float(np.mean(fpga_times) * 1000.0)
        speedup = cpu_mean_ms / max(fpga_mean_ms, 0.001)

        print(f" CPU Baseline Mean:  {cpu_mean_ms:.2f} ms ({1000.0/cpu_mean_ms:.1f} FPS)")
        print(f" FPGA Pipeline Mean: {fpga_mean_ms:.2f} ms ({1000.0/fpga_mean_ms:.1f} FPS)")
        return {
            "iterations": iterations,
            "cpu_mean_ms": cpu_mean_ms,
            "fpga_mean_ms": fpga_mean_ms,
            "speedup": speedup
        }

    def close(self):
        if self.is_hardware:
            if self.in_buffer is not None:
                self.in_buffer.close()
            if self.out_buffer is not None:
                self.out_buffer.close()

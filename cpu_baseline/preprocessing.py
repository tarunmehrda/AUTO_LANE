"""
preprocessing.py
----------------
Module: Classical Computer Vision Preprocessing Pipeline
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

This module provides verified software reference functions for:
1. Color to Grayscale conversion (Standard OpenCV & Bit-Accurate Fixed-Point FPGA Model)
2. Gaussian Blur smoothing (Continuous & Hardware 3x3 Shift-and-Add Model)
3. Sobel Edge Gradient calculation (Gx, Gy, Exact & L1 Hardware Approximation)
4. Configurable Binary Thresholding
5. Region of Interest (ROI) Masking
"""

import cv2
import numpy as np


def bgr_to_grayscale(image_bgr: np.ndarray, hardware_emulation: bool = False) -> np.ndarray:
    """
    Convert an 8-bit BGR image to single-channel 8-bit Grayscale.

    Engineering Details:
    - Standard OpenCV uses floating-point coefficients:
        Y = 0.299*R + 0.587*G + 0.114*B
    - Hardware PL (FPGA) avoids expensive floating-point arithmetic.
      Instead, it uses integer fixed-point coefficients scaled by 256 (2^8):
        77/256 ≈ 0.3008 (Red)
        150/256 ≈ 0.5859 (Green)
        29/256 ≈ 0.1133 (Blue)
        Sum: 77 + 150 + 29 = 256.
      In FPGA RTL, division by 256 is a simple 8-bit right shift: >> 8.
    """
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("Input image is empty or invalid.")

    if not hardware_emulation:
        return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    else:
        # Fixed-point integer arithmetic identical to FPGA Verilog logic
        b = image_bgr[:, :, 0].astype(np.uint32)
        g = image_bgr[:, :, 1].astype(np.uint32)
        r = image_bgr[:, :, 2].astype(np.uint32)
        
        # (77*R + 150*G + 29*B) >> 8
        gray_hw = (77 * r + 150 * g + 29 * b) >> 8
        return np.clip(gray_hw, 0, 255).astype(np.uint8)


def gaussian_blur(image_gray: np.ndarray, kernel_size: int = 5, sigma: float = 1.0, 
                  hardware_emulation: bool = False) -> np.ndarray:
    """
    Apply 2D Gaussian smoothing to reduce sensor noise and surface road texture.

    Engineering Details:
    - Standard OpenCV applies a Gaussian kernel with continuous weights.
    - Hardware PL utilizes a separable 3x3 integer kernel with powers-of-two weights:
        [1, 2, 1] / 4 in X and [1, 2, 1] / 4 in Y
        Equivalent 2D 3x3 kernel:
            [ 1  2  1 ]
            [ 2  4  2 ] / 16
            [ 1  2  1 ]
      Division by 16 is implemented in RTL as a 4-bit right shift: >> 4.
    """
    if not hardware_emulation:
        if kernel_size % 2 == 0:
            raise ValueError(f"Kernel size must be odd, got {kernel_size}")
        return cv2.GaussianBlur(image_gray, (kernel_size, kernel_size), sigma)
    else:
        # Hardware shift-and-add 3x3 integer kernel
        kernel_hw = np.array([
            [1, 2, 1],
            [2, 4, 2],
            [1, 2, 1]
        ], dtype=np.int32)
        
        # Convolve using filter2D with exact integer weights and bit-shift division
        convolved = cv2.filter2D(image_gray.astype(np.int32), -1, kernel_hw)
        blurred_hw = convolved >> 4
        return np.clip(blurred_hw, 0, 255).astype(np.uint8)


def sobel_gradients(image_gray: np.ndarray, ksize: int = 3, 
                    hardware_emulation: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute directional Sobel gradients Gx, Gy and overall gradient magnitude.

    Sobel Kernels:
        Gx = [ -1  0  +1 ]        Gy = [ -1  -2  -1 ]
             [ -2  0  +2 ]             [  0   0   0 ]
             [ -1  0  +1 ]             [ +1  +2  +1 ]

    Engineering Details:
    - Gx highlights vertical edges (lane lines).
    - Gy highlights horizontal edges (speed breakers, road markings, crosswalks).
    - Magnitude:
        Exact: sqrt(Gx^2 + Gy^2)
        Hardware Approximation (L1 Manhattan norm): |Gx| + |Gy|
        Why L1 in FPGA? Square roots and squares require multi-cycle DSP48 slices and latency.
        |Gx| + |Gy| eliminates DSP usage for magnitude and is standard for real-time edge detection.
    """
    # Compute signed 16-bit gradients to prevent signed underflow/overflow
    grad_x = cv2.Sobel(image_gray, cv2.CV_16S, 1, 0, ksize=ksize)
    grad_y = cv2.Sobel(image_gray, cv2.CV_16S, 0, 1, ksize=ksize)

    if not hardware_emulation:
        # L2 norm (Euclidean)
        mag_float = np.sqrt(grad_x.astype(np.float32)**2 + grad_y.astype(np.float32)**2)
        magnitude = np.clip(mag_float, 0, 255).astype(np.uint8)
    else:
        # L1 norm (|Gx| + |Gy|) matching FPGA pipeline
        abs_x = np.abs(grad_x)
        abs_y = np.abs(grad_y)
        mag_l1 = abs_x + abs_y
        magnitude = np.clip(mag_l1, 0, 255).astype(np.uint8)

    return grad_x, grad_y, magnitude


def apply_threshold(magnitude: np.ndarray, threshold_value: int) -> np.ndarray:
    """
    Convert gradient magnitude to a binary edge mask.
    Pixels >= threshold_value become 255 (white); otherwise 0 (black).
    """
    _, binary_edges = cv2.threshold(
        magnitude, 
        thresh=threshold_value, 
        maxval=255, 
        type=cv2.THRESH_BINARY
    )
    return binary_edges


def apply_roi_mask(image: np.ndarray, polygon_vertices: np.ndarray = None) -> np.ndarray:
    """
    Mask out non-road areas (sky, trees, oncoming traffic, vehicle hood).
    
    If polygon_vertices is None, defaults to a trapezoidal road corridor for a 640x360 frame.
    """
    h, w = image.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)

    if polygon_vertices is None:
        # Default trapezoid normalized coordinates
        polygon_vertices = np.array([
            [int(0.10 * w), int(0.95 * h)],  # Bottom-left
            [int(0.42 * w), int(0.60 * h)],  # Top-left (horizon)
            [int(0.58 * w), int(0.60 * h)],  # Top-right (horizon)
            [int(0.90 * w), int(0.95 * h)]   # Bottom-right
        ], dtype=np.int32)

    # Fill polygon with 255 (valid road area)
    cv2.fillPoly(mask, [polygon_vertices], 255)

    # Bitwise AND to keep only edges inside the ROI
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    return masked_image

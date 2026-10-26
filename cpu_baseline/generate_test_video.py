"""
generate_test_video.py
-----------------------
Utility to generate a realistic 100-frame multi-condition Indian road video.
Conditions covered:
- Frames 01-25: Clear road with crisp white & yellow lane markings.
- Frames 26-50: Faded road markings with a painted thermoplastic speed breaker.
- Frames 51-75: Unmarked rural road with soft dirt/vegetation shoulders (Virtual Lane).
- Frames 76-100: Curved highway returning to marked pavement with lateral vehicle sway.
"""

import os
import sys
import numpy as np
import cv2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from cpu_baseline.test_synthetic_pipeline import generate_synthetic_road_image
from cpu_baseline.test_confidence_threshold import generate_faded_road_frame
from cpu_baseline.test_virtual_lane import generate_unmarked_rural_road
from cpu_baseline.test_lane_detection import generate_curved_road_frame
from cpu_baseline.test_speed_breaker import generate_speed_breaker_scene


def build_composite_test_video(output_path: str, width: int = 640, height: int = 360, fps: float = 30.0):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    print(f"[Video Generator] Rendering 100-frame multi-condition video to: {output_path}")

    for frame_idx in range(100):
        # Section 1: Crisp lane markings (Frames 0-24)
        if frame_idx < 25:
            # Add slight vehicle swaying
            sway = int(np.sin(frame_idx * 0.25) * 8.0)
            base = generate_synthetic_road_image(width, height)
            frame = np.roll(base, sway, axis=1)

        # Section 2: Faded markings + Speed breaker approaching (Frames 25-49)
        elif frame_idx < 50:
            rel_idx = frame_idx - 25
            fade = 0.82
            base = generate_faded_road_frame(width, height, fade_level=fade)
            # Add speed breaker in frames 30 to 45 moving down from y = 230 to 310
            if 30 <= frame_idx <= 45:
                sb_y = 230 + int((frame_idx - 30) * 5.0)
                # Draw 3 horizontal thermoplastic bars
                for b_i in range(3):
                    bar_y = sb_y + (b_i * 24)
                    if bar_y + 8 < height:
                        bar_col = (0, 215, 255) if (b_i % 2 == 0) else (245, 245, 245)
                        cv2.rectangle(base, (width // 2 - 120, bar_y), (width // 2 + 120, bar_y + 8), bar_col, -1)
            frame = base

        # Section 3: Unmarked rural road (Frames 50-74)
        elif frame_idx < 75:
            base = generate_unmarked_rural_road(width, height)
            sway = int(np.sin(frame_idx * 0.15) * 6.0)
            frame = np.roll(base, sway, axis=1)

        # Section 4: Curved marked road (Frames 75-99)
        else:
            rel_idx = frame_idx - 75
            curve_dir = 1.0 if rel_idx < 15 else -1.0
            frame = generate_curved_road_frame(width, height, curve_direction=curve_dir)

        writer.write(frame)

    writer.release()
    print(f"[Video Generator] Successfully saved {output_path} ({os.path.getsize(output_path)} bytes)")
    return output_path


if __name__ == "__main__":
    out_file = os.path.join(PROJECT_ROOT, "data", "raw_videos", "indian_road_composite_test.mp4")
    build_composite_test_video(out_file)

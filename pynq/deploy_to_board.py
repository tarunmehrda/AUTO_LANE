"""
deploy_to_board.py
------------------
Automated deployment script to transfer the AUTO_LANE project files
from host workstation to the physical PYNQ-Z2 board via SSH / SCP.
"""

import os
import sys
import paramiko
from scp import SCPClient

BOARD_IP = '192.168.2.99'
BOARD_USER = 'xilinx'
BOARD_PASS = 'xilinx'
REMOTE_DIR = '/home/xilinx/jupyter_notebooks/auto_lane'

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)

def deploy():
    print("=" * 65)
    print("  AUTOMATED DEPLOYMENT TO PYNQ-Z2 BOARD (192.168.2.99)")
    print("=" * 65)

    print(f"[1/4] Connecting to {BOARD_IP} via SSH...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(BOARD_IP, username=BOARD_USER, password=BOARD_PASS, timeout=10)
    print("      Connected successfully as user 'xilinx'.")

    print(f"[2/4] Creating remote project directory: {REMOTE_DIR}...")
    ssh.exec_command(f"mkdir -p {REMOTE_DIR}/cpu_baseline {REMOTE_DIR}/config {REMOTE_DIR}/data/raw_videos {REMOTE_DIR}/pynq/notebooks {REMOTE_DIR}/pynq/overlays")

    print("[3/4] Uploading project files to PYNQ-Z2...")
    with SCPClient(ssh.get_transport()) as scp:
        # 1. Config
        config_src = os.path.join(PROJECT_ROOT, "config", "config.json")
        scp.put(config_src, f"{REMOTE_DIR}/config/config.json")
        print("      Uploaded: config/config.json")

        # 2. CPU Baseline / Algorithms
        cpu_dir = os.path.join(PROJECT_ROOT, "cpu_baseline")
        for f in os.listdir(cpu_dir):
            if f.endswith(".py"):
                local_path = os.path.join(cpu_dir, f)
                scp.put(local_path, f"{REMOTE_DIR}/cpu_baseline/{f}")
        print("      Uploaded: cpu_baseline algorithms (*.py)")

        # 3. PYNQ driver & demo notebook
        driver_src = os.path.join(PROJECT_ROOT, "pynq", "hardware_test.py")
        scp.put(driver_src, f"{REMOTE_DIR}/pynq/hardware_test.py")
        
        nb_src = os.path.join(PROJECT_ROOT, "pynq", "notebooks", "01_lane_detection_fpga_demo.ipynb")
        scp.put(nb_src, f"{REMOTE_DIR}/01_lane_detection_fpga_demo.ipynb")
        scp.put(nb_src, f"{REMOTE_DIR}/pynq/notebooks/01_lane_detection_fpga_demo.ipynb")
        print("      Uploaded: 01_lane_detection_fpga_demo.ipynb & hardware_test.py")

        # 4. Test Video
        video_src = os.path.join(PROJECT_ROOT, "data", "raw_videos", "indian_road_composite_test.mp4")
        if os.path.exists(video_src):
            scp.put(video_src, f"{REMOTE_DIR}/data/raw_videos/indian_road_composite_test.mp4")
            print("      Uploaded: data/raw_videos/indian_road_composite_test.mp4")

    print("[4/4] Verifying remote files...")
    stdin, stdout, stderr = ssh.exec_command(f"ls -la {REMOTE_DIR}")
    print(stdout.read().decode())

    ssh.close()
    print("=" * 65)
    print("  DEPLOYMENT COMPLETE! Project is live on PYNQ-Z2 board!")
    print("=" * 65)

if __name__ == "__main__":
    deploy()

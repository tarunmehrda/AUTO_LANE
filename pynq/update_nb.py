import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.2.99', username='xilinx', password='xilinx', timeout=10)

cmd = (
    "python3 -c \""
    "p = '/home/xilinx/jupyter_notebooks/auto_lane/01_lane_detection_fpga_demo.ipynb'; "
    "c = open(p).read().replace('iterations=5', 'iterations=2'); "
    "open(p, 'w').write(c); "
    "print('NOTEBOOK_UPDATED_SUCCESS');"
    "\""
)

stdin, stdout, stderr = ssh.exec_command(cmd)
print("OUT:", stdout.read().decode())
print("ERR:", stderr.read().decode())
ssh.close()

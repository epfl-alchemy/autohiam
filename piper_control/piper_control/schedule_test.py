import subprocess
import time
import os
import signal
import sys

# print("Opening the marker detection node")
# cmd5 = """
# source ~/directory_env/handeye_env/bin/activate
# cd ~/autohiam_ws
# source install/setup.bash
# ros2 run marker_detection gripper_estimate_marker_pose 
# """

# # Start the process in a new process group
# marker_detection_node = subprocess.Popen(
#     cmd5,
#     shell=True,
#     executable="/bin/bash",
#     stdout=subprocess.DEVNULL,
#     stderr=subprocess.DEVNULL,
#     preexec_fn=os.setsid  # Create new process group
# )

# # Wait for startup
# for i in range(10, 0, -1):
#     sys.stdout.write(f"\rWaiting for initialization: {i} seconds remaining... ")
#     sys.stdout.flush()
#     time.sleep(1)

# # Kill the whole process group (gracefully)
# print("\nShutting down marker detection node...")
# os.killpg(os.getpgid(marker_detection_node.pid), signal.SIGINT)

# # Optionally wait for cleanup
# marker_detection_node.wait()
print("Starting cumotion...")
cmd8 = """
source ~/directory_env/curobo_env/bin/activate
cd ~/autohiam_ws
source install/setup.bash
ros2 run piper_control cumotion
"""
cumotion_node = subprocess.Popen(cmd8, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
# for i in range(8, 0, -1):
#     sys.stdout.write(f"Waiting: \r{i} seconds remaining... ")
#     sys.stdout.flush()
#     time.sleep(1)

print("Opening the marker detection node")
cmd5 = """
source ~/directory_env/handeye_env/bin/activate
cd ~/autohiam_ws
source install/setup.bash
ros2 run marker_detection gripper_estimate_marker_pose 
"""
marker_detection_node = subprocess.Popen(cmd5, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

subprocess.run("pkill -f cumotion", shell=True)
subprocess.run("pkill -f realsense2_camera", shell=True)
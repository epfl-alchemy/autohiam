import subprocess
import time
import sys
import signal
import os

all_procs = []

def wait_seconds(seconds, message=""):
    for i in range(seconds, 0, -1):
        sys.stdout.write(f"{message}\r{i} seconds remaining... ")
        sys.stdout.flush()
        time.sleep(1)

def keep_enable_flag(seconds=5, state=True):
    cmd = f'ros2 topic pub -r 50 /enable_flag std_msgs/msg/Bool "{{data: {str(state).lower()}}}"'
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(seconds)
    proc.terminate()
    proc.wait()

def keep_disable_flag(seconds=5, state=False):
    cmd = f'ros2 topic pub -r 50 /enable_flag std_msgs/msg/Bool "{{data: {str(state).lower()}}}"'
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(seconds)
    proc.terminate()
    proc.wait()

try: 
    print("Running can_activate.sh...")
    cmd_can = """
    cd ~/piper_ros_ws/src/piper_ros
    bash can_activate.sh can0 1000000
    """
    subprocess.run(cmd_can, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("can_activate.sh executed.")
    #####################
    print("starting the robot...")
    cmd_enable = """
    source ~/directory_env/piper_env/bin/activate
    cd ~/piper_ros_ws
    source install/setup.bash
    ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2
    """
    robot_control_node = subprocess.Popen(cmd_enable, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid) #Run the commands in an interactive shell
    all_procs.append(robot_control_node)
    wait_seconds(5, "waiting")
    print("robot control node has started.")
    ####################
    cmd_moveit = """
    source ~/directory_env/piper_env/bin/activate
    cd ~/piper_ros_ws
    source install/setup.bash
    ros2 launch piper_with_gripper_moveit demo.launch.py
    """
    moveit_node = subprocess.Popen(cmd_moveit, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    all_procs.append(moveit_node)
    wait_seconds(5, "waiting")
    print("The robot is now ready for AUTOHIAM.")


    keep_disable_flag()
    wait_seconds(5, "waiting")
    keep_enable_flag()

except subprocess.CalledProcessError as e:
    print(f"Command failed: {e}")
    print("\nShutting down everything...")
    for proc in all_procs:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception as e:
            print(f"Could not kill process {proc}: {e}")

    print("Killing old nodes if any...")

    # Kill common ROS 2 processes launched by ros2 run / ros2 launch
    subprocess.run("sudo pkill -f piper_single_ctrl", shell=True)
    subprocess.run("sudo pkill -f moveit_node", shell=True)

    # Kill any leftover ros2 nodes or launch processes
    subprocess.run("sudo pkill -f ros2", shell=True)

    # Optionally kill RViz if used (not in your current script but just in case)
    subprocess.run("sudo pkill -f rviz", shell=True)

finally:
    print("\nShutting down everything...")
    for proc in all_procs:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception as e:
            print(f"Could not kill process {proc}: {e}")

    print("Killing old nodes if any...")

    # Kill common ROS 2 processes launched by ros2 run / ros2 launch
    subprocess.run("sudo pkill -f piper_single_ctrl", shell=True)
    subprocess.run("sudo pkill -f moveit_node", shell=True)

    # Kill any leftover ros2 nodes or launch processes
    subprocess.run("sudo pkill -f ros2", shell=True)

    # Optionally kill RViz if used (not in your current script but just in case)
    subprocess.run("sudo pkill -f rviz", shell=True)
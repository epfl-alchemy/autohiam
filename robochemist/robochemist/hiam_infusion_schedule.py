import subprocess
import time
import sys
import signal
import os

all_procs = []

# === Robochemist workspace commands ===
cmd_open_gripper = "cd ~/autohiam_ws && source install/setup.bash && ros2 run robochemist open_gripper"
cmd_close_gripper = "cd ~/autohiam_ws && source install/setup.bash && ros2 run robochemist close_gripper"
cmd_zero_gripper = "cd ~/autohiam_ws && source install/setup.bash && ros2 run robochemist zero_gripper"

cmd_disable_pose = "cd ~/autohiam_ws && source install/setup.bash && ros2 run robochemist disable_pose"
cmd_start_pose = "cd ~/autohiam_ws && source install/setup.bash && ros2 run robochemist start_pose"

cmd_shaking = "cd ~/autohiam_ws && source install/setup.bash && ros2 run robochemist shaking"

cmd_pickup_container = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist pickup"
)
cmd_pickup_iron_cover = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist pickup --ros-args -p marker_id:=23"
)

cmd_cartesian_move_down = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist cartesian_control_moveit --ros-args -p z_offset:=-0.07"
)
cmd_cartesian_move_down_more = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist cartesian_control_moveit --ros-args -p z_offset:=-0.09"
)

cmd_cartesian_move_up = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist cartesian_control_moveit --ros-args -p z_offset:=0.10"
)

cmd_move_to_heater = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist heater"
)
cmd_move_to_ammonia = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist moveto"
)
cmd_move_to_water = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist moveto --ros-args -p y:=0.103"
)
cmd_move_to_cover_base = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist moveto --ros-args -p y:=0.000"
)


# === Cumotion (control logic) ===
cmd_run_cumotion = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run robochemist cumotion"
)

# === Realsense and vision ===
cmd_can_activate = "cd ~/piper_ros_ws/src/piper_ros && bash can_activate.sh can0 1000000"
cmd_realsense_node = (
    "ros2 run realsense2_camera realsense2_camera_node "
    "--ros-args -p enable_color:=true -p rgb_camera.color_profile:=1920x1080x30"
)
cmd_marker_detection = (
    "source ~/directory_env/handeye_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run marker_detection gripper_estimate_marker_pose"
)

# === Robot control (Piper ROS) ===
cmd_enable_robot = (
    "source ~/directory_env/piper_env/bin/activate && "
    "cd ~/piper_ros_ws && source install/setup.bash && "
    "ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2"
)
cmd_send_enable_command = (
    "ros2 topic pub -r 100 /enable_flag std_msgs/msg/Bool '{data: true}'"
)
cmd_send_disable_command = (
    "ros2 topic pub -r 50 /enable_flag std_msgs/msg/Bool '{data: false}'"
)
cmd_launch_moveit = (
    "source ~/directory_env/piper_env/bin/activate && "
    "cd ~/piper_ros_ws && source install/setup.bash && "
    "ros2 launch piper_with_gripper_moveit demo.launch.py"
)

def wait_seconds(seconds, message=""):
    for i in range(seconds, 0, -1):
        sys.stdout.write(f"{message}\r{i} seconds remaining... ")
        sys.stdout.flush()
        time.sleep(1)

def keep_disable_flag(seconds=5, state=False):
    cmd = f"ros2 topic pub -r 50 /enable_flag std_msgs/msg/Bool '{{data: {str(state).lower()}}}'"
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(seconds)
    proc.terminate()
    proc.wait()

cleanup_done = False
send_enable_node = None

def cleanup():
    global cleanup_done
    if cleanup_done:
        print("Cleanup already completed. Skipping...")
        return
    cleanup_done = True

    print("Starting cleanup...")

    try:
        print("Opening the gripper...")
        subprocess.run(cmd_open_gripper, shell=True, executable="/bin/bash", check=True)

        print("Moving up...")
        subprocess.run(cmd_cartesian_move_up, shell=True, executable="/bin/bash", check=True)

        print("Going to disable pose...")
        subprocess.run(cmd_disable_pose, shell=True, executable="/bin/bash", check=True)

        print("Closing the gripper to zero...")
        subprocess.run(cmd_zero_gripper, shell=True, executable="/bin/bash", check=True)

        print("Shutting down the robot...")

        if send_enable_node is not None:
            send_enable_node.terminate()
            send_enable_node.wait()
        keep_disable_flag()

    except Exception as e:
        print(f"Error during shutdown: {e}")

    print("\nShutting down everything...")
    for proc in all_procs:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception as e:
            print(f"Could not kill process {proc}: {e}")

    subprocess.run("sudo pkill -f gripper_estimate_marker_pose", shell=True)
    subprocess.run("sudo pkill -f piper_single_ctrl", shell=True)
    subprocess.run("sudo pkill -f cumotion", shell=True)
    subprocess.run("sudo pkill -f realsense2_camera", shell=True)
    subprocess.run("sudo pkill -f moveit_node", shell=True)
    subprocess.run("sudo pkill -f ros2", shell=True)

# Register signal handler
def signal_handler(sig, frame):
    print("\nCtrl+C detected!")
    cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def initialization():
    global send_enable_node
    print("Running can_activate.sh...")
    subprocess.run(cmd_can_activate, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("can_activate.sh executed.")
    #####################
    print("Starting the camera control node...")
    realsense_node = subprocess.Popen(cmd_realsense_node, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    all_procs.append(realsense_node)

    wait_seconds(5, "waiting")

    print("Opening the marker detection node")
    marker_detection_node = subprocess.Popen(cmd_marker_detection, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    all_procs.append(marker_detection_node)
    wait_seconds(5, "waiting")

    print("starting the robot...")
    robot_control_node = subprocess.Popen(cmd_enable_robot, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid) #Run the commands in an interactive shell
    all_procs.append(robot_control_node)
    wait_seconds(5, "waiting")
    # send_enable_node = subprocess.Popen(cmd_send_enable_command, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid) #Run the commands in an interactive shell
    # all_procs.append(send_enable_node)
    print("robot control node has started.")

    moveit_node = subprocess.Popen(cmd_launch_moveit, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    all_procs.append(moveit_node)
    wait_seconds(5, "waiting")
    print("The robot is now ready for AUTOHIAM.")

    print("Starting cumotion...")
    cumotion_node = subprocess.Popen(cmd_run_cumotion, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    all_procs.append(cumotion_node)
    wait_seconds(10, "waiting for cumotion")

def infusion():
    print("=== Step 1: Infusion ===")

    print("Looking for cover on the heater...")
    subprocess.run(cmd_start_pose, shell=True, executable="/bin/bash", check=True)

    print("Opening the gripper...")
    subprocess.run(cmd_open_gripper, shell=True, executable="/bin/bash", check=True)

    print("Picking up the iron cover...")
    subprocess.run(cmd_pickup_iron_cover, shell=True, executable="/bin/bash", check=True)

    print("Moving down...")
    subprocess.run(cmd_cartesian_move_down, shell=True, executable="/bin/bash", check=True)

    print("Closing the gripper...")
    subprocess.run(cmd_close_gripper, shell=True, executable="/bin/bash", check=True)

    print("Moving up...")
    subprocess.run(cmd_cartesian_move_up, shell=True, executable="/bin/bash", check=True)

    print("Moving the cover to the base...")
    subprocess.run(cmd_move_to_cover_base, shell=True, executable="/bin/bash", check=True)

    print("Moving down...")
    subprocess.run(cmd_cartesian_move_down_more, shell=True, executable="/bin/bash", check=True)

    print("Opening the gripper...")
    subprocess.run(cmd_open_gripper, shell=True, executable="/bin/bash", check=True)

    print("Moving up...")
    subprocess.run(cmd_cartesian_move_up, shell=True, executable="/bin/bash", check=True)

    print("Moving to start position...")
    subprocess.run(cmd_start_pose, shell=True, executable="/bin/bash", check=True)

    print("Picking up the sample...")
    subprocess.run(cmd_pickup_container, shell=True, executable="/bin/bash", check=True)

    print("Moving down...")
    subprocess.run(cmd_cartesian_move_down, shell=True, executable="/bin/bash", check=True)

    print("Closing the gripper...")
    subprocess.run(cmd_close_gripper, shell=True, executable="/bin/bash", check=True)

    print("Moving up...")
    subprocess.run(cmd_cartesian_move_up, shell=True, executable="/bin/bash", check=True)

    print("Shaking...")
    subprocess.run(cmd_shaking, shell=True, executable="/bin/bash", check=True)

    print("Going to dry...")
    subprocess.run(cmd_start_pose, shell=True, executable="/bin/bash", check=True)

    print("Shaking...")
    subprocess.run(cmd_shaking, shell=True, executable="/bin/bash", check=True)

    wait_seconds(5, "waiting to dry")

    print("Moving into the heater...")
    subprocess.run(cmd_move_to_heater, shell=True, executable="/bin/bash", check=True)

    print("Moving straight down...")
    subprocess.run(cmd_cartesian_move_down_more, shell=True, executable="/bin/bash", check=True)

if __name__ == "__main__":
    try: 
        initialization()
        infusion()
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {e}")
        cleanup()
    finally:
        cleanup()
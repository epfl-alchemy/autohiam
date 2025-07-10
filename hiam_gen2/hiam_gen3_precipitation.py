import subprocess
import time
import sys
import signal
import os

all_procs = []

# === Robochemist workspace commands ===
cmd_open_gripper = "cd ~/autohiam_ws && source install/setup.bash && ros2 run hiam_gen2 open_gripper"
cmd_close_gripper = "cd ~/autohiam_ws && source install/setup.bash && ros2 run hiam_gen2 close_gripper"
cmd_zero_gripper = "cd ~/autohiam_ws && source install/setup.bash && ros2 run hiam_gen2 zero_gripper"

cmd_disable_pose = "cd ~/autohiam_ws && source install/setup.bash && ros2 run hiam_gen2 disable_pose"
cmd_start_pose = "cd ~/autohiam_ws && source install/setup.bash && ros2 run hiam_gen2 start_pose"

cmd_shaking = "cd ~/autohiam_ws && source install/setup.bash && ros2 run hiam_gen2 shaking"

cmd_pickup_container = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 pickup"
)
cmd_cartesian_move_up = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 cartesian_control_moveit --ros-args -p z_offset:=0.15"
)
cmd_move_to_dry = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 moveto --ros-args -p x:=0.488 -p y:=0.080"
)
cmd_move_to_dry_infusion = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 moveto --ros-args -p x:=0.253 -p y:=-0.143 -p z:=0.280"
)
cmd_move_to_water = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 moveto --ros-args -p x:=0.375 -p y:=0.078"
)
cmd_move_to_precipitation = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 moveto_marker --ros-args -p marker_id:=55 -p z:=0.250"
)

cmd_cartesian_move_down = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 cartesian_control_moveit --ros-args -p z_offset:=-0.13"
)
cmd_cartesian_move_down_precipitation = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 cartesian_control_moveit --ros-args -p z_offset:=-0.12"
)

cmd_pickup_infusion_cover = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 pickup --ros-args -p marker_id:=23"
)
cmd_move_to_infusion_cover_base = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 moveto --ros-args -p x:=0.3125 -p y:=-0.0125 -p z:=0.16"
)

cmd_move_to_heater = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 moveto_marker"
)
cmd_move_to_origin = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 moveto --ros-args -p x:=0.488 -p y:=0.180"
)
cmd_move_cover_to_heater = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 moveto_marker --ros-args -p z:=0.11"
)

cmd_cartesian_move_down_heater = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 cartesian_control_moveit --ros-args -p z_offset:=-0.11"
)


cmd_cartesian_move_down_heater = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 cartesian_control_moveit --ros-args -p z_offset:=-0.10"
)

# === Cumotion (control logic) ===
cmd_run_cumotion = (
    "source ~/directory_env/curobo_env/bin/activate && "
    "cd ~/autohiam_ws && source install/setup.bash && "
    "ros2 run hiam_gen2 cumotion"
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
        subprocess.run(cmd_open_gripper, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        print("Moving up...")
        subprocess.run(cmd_cartesian_move_up, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        print("Going to disable pose...")
        subprocess.run(cmd_disable_pose, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        print("Closing the gripper to zero...")
        subprocess.run(cmd_zero_gripper, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

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
    subprocess.run("sudo pkill -f rviz", shell=True)

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
    print("robot control node has started.")

    moveit_node = subprocess.Popen(cmd_launch_moveit, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    all_procs.append(moveit_node)
    wait_seconds(5, "waiting")
    print("The robot is now ready for AUTOHIAM.")

    print("Starting cumotion...")
    cumotion_node = subprocess.Popen(cmd_run_cumotion, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    all_procs.append(cumotion_node)
    wait_seconds(10, "waiting for cumotion")


def precipitation():
    print("=== Step 2: Precipitation ===")

    print("Going to find sample...")
    subprocess.run(cmd_start_pose, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Opening the gripper...")
    subprocess.run(cmd_open_gripper, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Picking up the sample...")
    subprocess.run(cmd_pickup_container, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Closing the gripper...")
    subprocess.run(cmd_close_gripper, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving up...")
    subprocess.run(cmd_cartesian_move_up, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving to dry...")
    subprocess.run(cmd_move_to_dry_infusion, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving down...")
    subprocess.run(cmd_cartesian_move_down, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Shaking...")
    subprocess.run(cmd_shaking, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving up...")
    subprocess.run(cmd_cartesian_move_up, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving to find the precipitation solution...")
    subprocess.run(cmd_start_pose, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving to the precipitation...")
    subprocess.run(cmd_move_to_precipitation, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving down...")
    subprocess.run(cmd_cartesian_move_down_precipitation, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Opening the gripper...")
    subprocess.run(cmd_open_gripper, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving up...")
    subprocess.run(cmd_cartesian_move_up, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Going to find the infusion cover...")
    subprocess.run(cmd_start_pose, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Picking up iron cover...")
    subprocess.run(cmd_pickup_infusion_cover, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Closing the gripper...")
    subprocess.run(cmd_close_gripper, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving up...")
    subprocess.run(cmd_cartesian_move_up, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving to find the heater...")
    subprocess.run(cmd_start_pose, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Moving to the heater...")
    subprocess.run(cmd_move_cover_to_heater, shell=True, executable="/bin/bash", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    try: 
        initialization()
        precipitation()
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {e}")
        cleanup()
    finally:
        cleanup()
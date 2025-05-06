import subprocess
import time
import sys
import signal
import os

def wait_seconds(seconds, message=""):
    for i in range(seconds, 0, -1):
        sys.stdout.write(f"{message}\r{i} seconds remaining... ")
        sys.stdout.flush()
        time.sleep(1)

# ros2 topic pub -r 10 /enable_flag std_msgs/msg/Bool "{data: true}"


try:
#####################
#Initialization
#####################
    print("Running can_activate.sh...")
    cmd_can = """
    cd ~/piper_ros_ws/src/piper_ros
    bash can_activate.sh can0 1000000
    """
    # robo1 = subprocess.run(
    #     ["bash", "can_activate.sh", "can0", "1000000"],
    #     cwd=os.path.expanduser("~/piper_ros_ws/src/piper_ros"),
    #     check=True
    # )
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
    wait_seconds(5, "waiting")
    print("robot control node has started.")

    cmd_keep_enable = """
    ros2 topic pub -r 50 /enable_flag std_msgs/msg/Bool "{data: true}"
    """
    keep_enable = subprocess.Popen(cmd_keep_enable, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    ####################
    cmd_moveit = """
    source ~/directory_env/piper_env/bin/activate
    cd ~/piper_ros_ws
    source install/setup.bash
    ros2 launch piper_with_gripper_moveit demo.launch.py
    """
    moveit_node = subprocess.Popen(cmd_moveit, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    wait_seconds(5, "waiting")
    print("The robot is now ready for AUTOHIAM.")
    #####################
    print("Starting the camera control node...")
    cmd_realsense = """
    ros2 run realsense2_camera realsense2_camera_node \
    --ros-args \
    -p enable_color:=true \
    -p rgb_camera.color_profile:=1280x720x30
    """
    realsense_node = subprocess.Popen(cmd_realsense, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    wait_seconds(5, "waiting")
    #####################
    print("Opening the marker detection node")
    cmd_marker = """
    source ~/directory_env/handeye_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run marker_detection gripper_estimate_marker_pose 
    """
    marker_detection_node = subprocess.Popen(cmd_marker, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    wait_seconds(5, "waiting")
    #####################
    print("Starting cumotion...")
    cmd_cumotion = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control cumotion
    """
    cumotion_node = subprocess.Popen(cmd_cumotion, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    wait_seconds(10, "waiting for cumotion")
#####################
#Infusion
#####################
    print("=== Step 1: Infusion ===")
    #####################
    print("Moving to start position...")
    cmd_start = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control start_position
    """
    subprocess.run(cmd_start, shell=True, executable="/bin/bash", check=True)
    
    #####################
    print("Opening the gripper...")
    cmd_open = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control open_gripper
    """
    # cmd_open = """
    # /home/szhuang/directory_env/piper_env/bin/python /home/szhuang/autohiam_ws/src/piper_control/piper_control/gripper_open_sdk.py    
    # """
    subprocess.run(cmd_open, shell=True, executable="/bin/bash", check=True)
    # wait_seconds(7, "waiting")
    #####################
    print("Picking up the container...")
    cmd_pickup = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control pick_up_sample
    """
    subprocess.run(cmd_pickup, shell=True, executable="/bin/bash", check=True)
    # wait_seconds(5, "waiting")
    #####################
    print("Moving down...")
    cmd_movedown = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control cartesian_control_moveit --ros-args -p z_offset:=-0.07
    """
    subprocess.run(cmd_movedown, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Closing the gripper...")
    cmd_close = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control close_gripper
    """
    # cmd_close = """
    # /home/szhuang/directory_env/piper_env/bin/python /home/szhuang/autohiam_ws/src/piper_control/piper_control/gripper_close_sdk.py
    # """
    subprocess.run(cmd_close, shell=True, executable="/bin/bash", check=True)
    # wait_seconds(7, "waiting")
    #####################
    print("Moving up...")
    cmd_moveup = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control cartesian_control_moveit --ros-args -p z_offset:=0.10
    """
    # ros2 run piper_control cartesian_control_client --ros-args -p z_offset:=0.10
    subprocess.run(cmd_moveup, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Going back to start position...")
    subprocess.run(cmd_start, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving into the heater...")
    cmd_heater = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control heater
    """
    subprocess.run(cmd_heater, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving straight down...")
    subprocess.run(cmd_movedown, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Opening the gripper...")
    subprocess.run(cmd_open, shell=True, executable="/bin/bash", check=True)
    # wait_seconds(7, "waiting")
    #####################
    print("Moving to start position...")
    subprocess.run(cmd_start, shell=True, executable="/bin/bash", check=True)
#####################
#Precipitation
#####################
    print("=== Step 2: Precipitation ===")
    #####################
    print("Picking up the container...")
    subprocess.run(cmd_pickup, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving straight down...")
    subprocess.run(cmd_movedown, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Closing the gripper...")
    subprocess.run(cmd_close, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving up...")
    subprocess.run(cmd_moveup, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Going back to start position...")
    subprocess.run(cmd_start, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving into the ammonia solution...")
    cmd_ammonia = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control move_into_beaker --ros-args -p marker_id:=100
    """
    subprocess.run(cmd_ammonia, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving straight down...")
    subprocess.run(cmd_movedown, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Opening the gripper...")
    subprocess.run(cmd_open, shell=True, executable="/bin/bash", check=True)
    # wait_seconds(7, "waiting")
    #####################
    print("Moving up...")
    subprocess.run(cmd_moveup, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving to start position...")
    subprocess.run(cmd_start, shell=True, executable="/bin/bash", check=True)
#####################
#Washing
#####################
    print("=== Step 3: Washing ===")
    #####################
    print("Picking up the container...")
    subprocess.run(cmd_pickup, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving straight down...")
    subprocess.run(cmd_movedown, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Closing the gripper...")
    subprocess.run(cmd_close, shell=True, executable="/bin/bash", check=True)
    # wait_seconds(7, "waiting")
    #####################
    print("Moving up...")
    subprocess.run(cmd_moveup, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Going back to start position...")
    subprocess.run(cmd_start, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving into the water...")
    cmd_water = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control move_into_beaker
    """
    subprocess.run(cmd_water, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving straight down...")
    subprocess.run(cmd_movedown, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Opening the gripper...")
    subprocess.run(cmd_open, shell=True, executable="/bin/bash", check=True)
    # wait_seconds(7, "waiting")
    #####################
    print("Moving up...")
    subprocess.run(cmd_moveup, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Moving to start position...")
    subprocess.run(cmd_start, shell=True, executable="/bin/bash", check=True)


finally:
#####################
#Ending the program
#####################
    print("Opening the gripper...")
    subprocess.run(cmd_open, shell=True, executable="/bin/bash", check=True)
    # wait_seconds(7, "waiting")
    #####################
    print("Going to zero position...")
    cmd_zero = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control zero
    """
    subprocess.run(cmd_zero, shell=True, executable="/bin/bash", check=True)
    #####################
    print("Closing the gripper...")
    # cmd_gripper_zero = """
    # /home/szhuang/directory_env/piper_env/bin/python /home/szhuang/autohiam_ws/src/piper_control/piper_control/gripper_zero_sdk.py
    # """
    cmd_gripper_zero = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control gripper_basic
    """
    subprocess.run(cmd_gripper_zero, shell=True, executable="/bin/bash", check=True)
    # wait_seconds(7, "waiting")
    #####################
    print("Shutting down the robot...")

    keep_enable.terminate()
    keep_enable.wait()
    wait_seconds(5, "waiting")

    cmd_disable = """
    ros2 topic pub -1 /enable_flag std_msgs/msg/Bool "data: false"
    """
    subprocess.run(cmd_disable, shell=True, executable="/bin/bash", check=True)
    print("\nShutting down everything...")
    for proc in [realsense_node, robot_control_node, marker_detection_node, moveit_node, cumotion_node]:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception as e:
            print(f"Could not kill process {proc}: {e}")

    print("Killing old nodes if any...")

    # Kill common ROS 2 processes launched by ros2 run / ros2 launch
    subprocess.run("sudo pkill -f gripper_estimate_marker_pose", shell=True)
    subprocess.run("sudo pkill -f piper_single_ctrl", shell=True)
    subprocess.run("sudo pkill -f cumotion", shell=True)
    subprocess.run("sudo pkill -f realsense2_camera", shell=True)
    subprocess.run("sudo pkill -f moveit_node", shell=True)

    # Kill any leftover ros2 nodes or launch processes
    subprocess.run("sudo pkill -f ros2", shell=True)

    # Optionally kill RViz if used (not in your current script but just in case)
    subprocess.run("sudo pkill -f rviz", shell=True)

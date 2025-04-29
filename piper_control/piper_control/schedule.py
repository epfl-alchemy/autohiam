import subprocess
import time
import sys
import signal
import os

#####################
#Initialization
#####################
try:
    print("Running can_activate.sh...")
    cmd1 = """
    cd ~/piper_ros_ws/src/piper_ros
    bash can_activate.sh can0 1000000
    """
    # robo1 = subprocess.run(
    #     ["bash", "can_activate.sh", "can0", "1000000"],
    #     cwd=os.path.expanduser("~/piper_ros_ws/src/piper_ros"),
    #     check=True
    # )
    subprocess.run(cmd1, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("can_activate.sh executed.")
    #####################
    print("starting the robot...")
    cmd2 = """
    source ~/directory_env/piper_env/bin/activate
    cd ~/piper_ros_ws
    source install/setup.bash
    ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2
    """
    robot_control_node = subprocess.Popen(cmd2, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid) #Run the commands in an interactive shell
    # for i in range(10, 0, -1):
    #     sys.stdout.write(f"Waiting for the robot control node to be started: \r{i} seconds remaining... ")
    #     sys.stdout.flush()
    #     time.sleep(1)
    print("robot control node has started.")
    ####################
    cmd3 = """
    source ~/directory_env/piper_env/bin/activate
    cd ~/piper_ros_ws
    source install/setup.bash
    ros2 launch piper_with_gripper_moveit demo.launch.py
    """
    moveit_node = subprocess.Popen(cmd3, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
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
    #####################
    print("Opening the marker detection node")
    cmd5 = """
    source ~/directory_env/handeye_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run marker_detection gripper_estimate_marker_pose 
    """
    marker_detection_node = subprocess.Popen(cmd5, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    #####################
    print("Starting cumotion...")
    cmd8 = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control cumotion
    """
    cumotion_node = subprocess.Popen(cmd8, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    for i in range(10, 0, -1):
        sys.stdout.write(f"Starting the AUTOHIAM process: \r{i} seconds remaining... ")
        sys.stdout.flush()
        time.sleep(1)
#####################
#Infusion
#####################
    #####################
    print("Moving to start position...")
    cmd6 = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control start_position
    """
    subprocess.run(cmd6, shell=True, executable="/bin/bash")
    #####################
    print("Opening the gripper...")
    cmd7 = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control open_gripper
    """
    subprocess.run(cmd7, shell=True, executable="/bin/bash")
    #####################
    print("Picking up the container...")
    cmd9 = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control pick_up_sample
    """
    subprocess.run(cmd9, shell=True, executable="/bin/bash")
    #####################
    print("Moving down...")
    cmd9 = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control cartesian_control_client --ros-args -p z_offset:=-0.08
    """
    subprocess.run(cmd9, shell=True, executable="/bin/bash")
    #####################
    print("Closing the gripper...")
    cmd10 = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control close_gripper
    """
    subprocess.run(cmd10, shell=True, executable="/bin/bash")
    #####################
    print("Moving up...")
    cmd9 = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control cartesian_control_client --ros-args -p z_offset:=0.09
    """
    subprocess.run(cmd9, shell=True, executable="/bin/bash")
    #####################
    print("Going back to start position...")
    cmd11 = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control start_position
    """
    subprocess.run(cmd11, shell=True, executable="/bin/bash")
    #####################
    print("Moving into the heater...")
    cmd12 = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control heater
    """
    #####################
    print("Moving straight down...")
    cmd13 = """
    source ~/directory_env/curobo_env/bin/activate
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control cartesian_control_client --ros-args -p z_offset:=-0.08
    """
    subprocess.run(cmd13, shell=True, executable="/bin/bash")
    #####################
    print("Opening the gripper...")
    cmd14 = """
    cd ~/autohiam_ws
    source install/setup.bash
    ros2 run piper_control open_gripper
    """
    subprocess.run(cmd14, shell=True, executable="/bin/bash")
#     #####################
#     print("Going to zero position...")
#     cmd15 = """
#     cd ~/autohiam_ws
#     source install/setup.bash
#     ros2 run piper_control zero
#     """
#     subprocess.run(cmd15, shell=True, executable="/bin/bash")
#     for i in range(5, 0, -1):
#         sys.stdout.write(f"Waiting: \r{i} seconds remaining... ")
#         sys.stdout.flush()
#         time.sleep(1)
#     #####################
#     print("Closing the gripper...")
#     cmd16 = """
#     cd ~/autohiam_ws
#     source install/setup.bash
#     ros2 run piper_control gripper_basic
#     """
#     subprocess.run(cmd16, shell=True, executable="/bin/bash")
#     for i in range(5, 0, -1):
#         sys.stdout.write(f"Waiting: \r{i} seconds remaining... ")
#         sys.stdout.flush()
#         time.sleep(1)
#     #####################
#     print("Shutting down the robot...")
#     cmd17 = """
#     ros2 topic pub -1 /enable_flag std_msgs/msg/Bool "data: false"
#     """
#     subprocess.run(cmd17, shell=True, executable="/bin/bash")


finally:
    print("\nShutting down everything...")
    for proc in [realsense_node, robot_control_node, marker_detection_node, moveit_node, handeye_node, cumotion_node]:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception as e:
            print(f"Could not kill process {proc}: {e}")

    print("Killing old nodes if any...")

    # Kill common ROS 2 processes launched by ros2 run / ros2 launch
    subprocess.run("pkill -f gripper_estimate_marker_pose", shell=True)
    subprocess.run("pkill -f piper_single_ctrl", shell=True)
    subprocess.run("pkill -f cumotion", shell=True)
    subprocess.run("pkill -f realsense2_camera", shell=True)
    subprocess.run("pkill -f handeye_calibration", shell=True)
    subprocess.run("pkill -f moveit_node", shell=True)

    # Kill any leftover ros2 nodes or launch processes
    subprocess.run("pkill -f ros2", shell=True)

    # Optionally kill RViz if used (not in your current script but just in case)
    subprocess.run("pkill -f rviz", shell=True)

#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import time
from piper_sdk import *

def enable_arm(piper: C_PiperInterface, timeout: int = 5):
    '''
    Enable the robot arm and check enable status.
    '''
    start_time = time.time()
    while True:
        # Try enabling
        piper.EnableArm(7)
        # Check all motors' enable status
        enable_status = all([
            piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status,
            piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status,
            piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status,
            piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status,
            piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status,
            piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status,
        ])
        if enable_status:
            print("Arm enabled successfully.")
            break
        if time.time() - start_time > timeout:
            print("Enable timeout. Exiting.")
            exit(1)
        time.sleep(0.5)

if __name__ == "__main__":
    # Initialize and connect
    piper = C_PiperInterface("can0")
    piper.ConnectPort()

    # Enable robot
    enable_arm(piper)

    # Open gripper (e.g., 50mm opening, adjust if needed)
    open_distance = 50 * 1000  # 50mm -> 50000
    force = 1000               # gripping force
    ctrl_mode = 0x01            # control mode
    reserved = 0                # reserved param (keep 0)
    
    piper.GripperCtrl(open_distance, force, ctrl_mode, reserved)
    print("Gripper opened.")

    # Optional: wait a bit to make sure command is executed
    time.sleep(1)

    print("Done.")

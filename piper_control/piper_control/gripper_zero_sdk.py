#!/usr/bin/env python3
from typing import (
    Optional,
)
import time
from piper_sdk import *

def enable_fun(piper:C_PiperInterface):
    '''
    Enable the robotic arm and check the enable status.
    Try for up to 5 seconds — exit the program if timeout occurs.
    '''
    enable_flag = False
    timeout = 5
    start_time = time.time()
    elapsed_time_flag = False
    while not (enable_flag):
        elapsed_time = time.time() - start_time
        print("--------------------")
        enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        print("Enable status:",enable_flag)
        piper.EnableArm(7)
        # piper.GripperCtrl(0,1000,0x01, 0)
        print("--------------------")

        if elapsed_time > timeout:
            print("Timeout...")
            elapsed_time_flag = True
            enable_flag = True
            break
        time.sleep(1)
        pass
    if(elapsed_time_flag):
        print("Automatic enable timeout, exiting program")
        exit(0)

if __name__ == "__main__":
    piper = C_PiperInterface("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper)

    piper.GripperCtrl(0, 1000, 0x01, 0)
    print("Gripper closed.")
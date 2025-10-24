#!/usr/bin/env python3
SDK_DIR_PARENT_PATH = "/home/atom/Workspaces/gr/sdk_challenge/ROBOT/"
import sys, os
sys.path.append(f"{SDK_DIR_PARENT_PATH}/ROBOT_SDK/ketirobotsdk")
from sdk import *
from time import *
import threading
import math
sys.path.append(f"{SDK_DIR_PARENT_PATH}/GRIPPER_SDK/include")
from zimmergripper import *

rob = Robot()

robot_connected = False
state = 0
cmd = 0
current_joint = []
current_T_matrix = []

gripper = KetiZimmer(f"{SDK_DIR_PARENT_PATH}/GRIPPER_SDK/lib/libzimmergripper.so")

setLibPath(f"{SDK_DIR_PARENT_PATH}/ROBOT_SDK/ketirobotsdk/librobotsdk.so")

def data_update_func():
	global robot_connected, state, cmd, current_joint, current_T_matrix
	while robot_connected is True:
		robot_info = rob.RobotInfo()

		current_joint = [robot_info.Jnt[0], robot_info.Jnt[1], robot_info.Jnt[2], robot_info.Jnt[3], robot_info.Jnt[4], robot_info.Jnt[5]]

		current_T_matrix = [
			robot_info.Mat[0], robot_info.Mat[1], robot_info.Mat[2], robot_info.Mat[3], 
			robot_info.Mat[4], robot_info.Mat[5], robot_info.Mat[6], robot_info.Mat[7], 
			robot_info.Mat[8], robot_info.Mat[9], robot_info.Mat[10], robot_info.Mat[11], 
			robot_info.Mat[12], robot_info.Mat[13], robot_info.Mat[14], robot_info.Mat[15]
			]

		if robot_info.State == 2:
			state = 2 #State.Moving
			cmd = 0
		elif robot_info.State == 1:
			state = 1 #State.Wait

		sleep(0.01)

rob.SetRobotConf(M1013, "192.168.137.101", 12345)
robot_connected = rob.RobotConnect()

gripper.Connect("192.168.137.201", 502)
gripper_connected = gripper.IsConnected()
print("wait...")
if gripper_connected is True:
    gripper.Init()
	
data_update_thread = threading.Thread(target=data_update_func, daemon=True)
data_update_thread.start()

try:
	while gripper_connected:
        print("current width : {0}".format(gripper.CurPos()))
        gripper.Grip()
        print("current width : {0}".format(gripper.CurPos()))
        gripper.Move(10)
        print("current width : {0}".format(gripper.CurPos()))
        gripper.Release()
        print("current width : {0}".format(gripper.CurPos()))
        gripper.Move(90)
        print("current width : {0}".format(gripper.CurPos()))
		
        sleep(0.5)
except KeyboardInterrupt:
	print("Ctrl+C")
finally:
	print("End.")
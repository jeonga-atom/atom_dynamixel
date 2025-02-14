#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author : Jeong A

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *
import math

portHandler = PortHandler('/dev/ttyACM0')
packetHandler = PacketHandler(2.0)

groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

portHandler.openPort()
portHandler.setBaudRate(1000000)




IDS                   = [1, 2, 3, 4]
IDS_2                 = [5, 6]

L = 0.3         # 휠베이스 (앞바퀴와 뒷바퀴 간 거리)
W = 0.2         # 트레드 (좌우 바퀴 간 거리)

def repeat_write (ID_list, addr_, data_val):
    if addr_ == 64 or addr_ == 11:
        for a in ID_list:
            packetHandler.write1ByteTxRx(portHandler, a , addr_, data_val)
    else:
        for b in ID_list:
            packetHandler.write4ByteTxRx(portHandler, b, addr_, data_val)

def arkermann_steering(R):                      # 조향각 계산
    theta_inner = math.degrees(math.atan(L / (R - W / 2)))   # 내륜 조향각
    theta_outer = math.degrees(math.atan(L / (R + W / 2)))   # 외륜 조향각
    return theta_inner, theta_outer

def set_wheel_angle(IDS_2, angle):                  # 각도를 Dynamixel 모터에 적용하는 함수
    goal_pos = int(angle * (4095 / 300))        # Dynamixel의 0~4095 값을 각도로 변환 (0~300도 범위)
    packetHandler.write2ByteTxRx(portHandler, IDS_2, 116, goal_pos)


repeat_write(IDS, 64, 0)  #setting for velocity(1,2,3,4)
repeat_write(IDS, 11, 1)
repeat_write(IDS, 64, 1)

repeat_write(IDS_2, 64, 0)  #setting for position(5,6)
repeat_write(IDS_2, 11, 3)
repeat_write(IDS_2, 64, 1)

goal_vel = 0
goal_pos = 2048

repeat_write(IDS_2, 116, goal_pos)

################################# main ##############################################


while True:
    char = getch()
    if char == 'w':  # press'w'= 앞으로
        goal_vel += 4
        print("present velocity: %d " %goal_vel)
        repeat_write (IDS, 104, goal_vel)
    
    elif char == 's':  # press's'= 뒤로
        goal_vel -= 8
        print("present velocity: %d " %goal_vel)
        repeat_write (IDS, 104, goal_vel)

   
    elif char == 'k':  # press'k'= 왼쪽으로 회전
        print("present position: %d " %goal_pos)
        goal_pos += 170
        angle_goal_pos = goal_pos * (360/4096)
        
        if goal_pos >= 2559:
            pass
        else:
            if goal_pos == 2048:
                goal_pos_outer == 2048
            else:
                repeat_write([5], 116, goal_pos)
                R = (L / math.tan(math.radians(angle_goal_pos - 180)) + (W / 2)) 
                goal_pos_outer = (math.degrees(math.atan(L / (R +(W / 2)))) + 180) * (4095 / 360)

                repeat_write([6], 116, int(goal_pos_outer))
            print(f"ID5= {goal_pos}, ID6= {goal_pos_outer}")


    elif char == 'l':  # press 'l'= 오른쪽으로 회전
        print("present position: %d " %goal_pos)
        goal_pos -= 170
        angle_goal_pos = goal_pos * (360/4096)
        
        if goal_pos <= 1535:
            pass
        else:
            if goal_pos == 2048:
                goal_pos_outer == 2048
            else:
                repeat_write([6], 116, goal_pos)
                R = (L / math.tan(math.radians(angle_goal_pos - 180)) + (W / 2)) 
                goal_pos_outer = (math.degrees(math.atan(L / (R +(W / 2)))) + 180) * (4095 / 360)

                repeat_write([5], 116, int(goal_pos_outer))
            print(f"ID5= {goal_pos}, ID6= {goal_pos_outer}")

    elif char == 'g': # 바퀴를 중앙으로 
        repeat_write (IDS_2, 116, 2048)

    elif char == '\x1b':  # ESC 키를 누르면 루프 종료
        print("Exiting loop...")
        repeat_write(IDS, 64, 0)
        break
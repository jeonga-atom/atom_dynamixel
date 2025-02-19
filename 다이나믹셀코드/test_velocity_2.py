#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author : Jeong A

######################## velocity control code #################################
########################## 아커만 공식 추가 ver ####################################
##########################일단은 vel_2말고 pos와 vel 합쳐서 만들기 시도###################

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


repeat_write(IDS, 64, 0)  #setting for velocity(1,2,3,4)
repeat_write(IDS, 11, 1)
repeat_write(IDS, 64, 1)

goal_vel = 0
######################################################################
################ 속도도 기본적으로 0으로 세팅하는 코드를 만들자 ################

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



    elif char == '\x1b':  # ESC 키를 누르면 루프 종료
        print("Exiting loop...")
        repeat_write(IDS, 64, 0)

        break
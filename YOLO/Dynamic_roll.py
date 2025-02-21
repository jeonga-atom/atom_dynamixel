
# import math
# import pyrealsense2 as rs
# import cv2
# import numpy as np
# import time

# from ultralytics import YOLO

# import sys, tty, termios
# fd = sys.stdin.fileno()
# old_settings = termios.tcgetattr(fd)
# def getch():
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return ch

# from dynamixel_sdk import *


# # YOLO 모델 로드
# model = YOLO('yolov8s.pt')

# # RealSense 파이프라인 설정
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.accel)
# config.enable_stream(rs.stream.gyro)

# align_to = rs.stream.color
# align = rs.align(align_to)

# #Dynamixel poter
# portHandler = PortHandler('/dev/ttyUSB0')
# packetHandler = PacketHandler(2.0)

# groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
# groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# portHandler.openPort()
# portHandler.setBaudRate(1000000)

# IDS                   = [1]

# packetHandler.write1ByteTxRx(portHandler, 1, 64, 0)
# packetHandler.write1ByteTxRx(portHandler,1, 11, 3)
# packetHandler.write1ByteTxRx(portHandler,1, 64, 1)

# # 스트리밍 시작
# pipeline.start(config)

# try:
#     while True:
#         # IMU 데이터 프레임 대기
#         frames = pipeline.wait_for_frames()
#         accel = frames.first_or_default(rs.stream.accel)
        
#         aligned_frames = align.process(frames)
#         color_frame = aligned_frames.get_color_frame()
        
#         color_image = np.asanyarray(color_frame.get_data())

#         results = model.track(source=color_image, persist=True, classes=39, verbose=False)

#         time.sleep(0.005)

#         if accel:
#             # Accelerometer 데이터 읽기
#             accel_data = accel.as_motion_frame().get_motion_data()
#             x, y, z = accel_data.x, accel_data.y, accel_data.z
            
#             # Roll 각도 계산 (라디안)
#             roll = math.atan2(-x, math.sqrt(y*y + z*z))
            
#             # 라디안을 도로 변환
#             roll_degrees = math.degrees(roll)
            
#             # print(f"Roll angle: {roll_degrees:.2f} degrees")
#             sys.stdout.write(f"\rRoll angle: {roll_degrees:.2f} degrees")

#             goal_position = 2048

            
#             if roll_degrees >0:
#                 goal_position -= roll_degrees * (4096/360)
#                 packetHandler.write4ByteTxRx(portHandler,1, 116, int(goal_position))

#             elif roll_degrees <0:
#                 goal_position -= roll_degrees * (4096/360)
#                 packetHandler.write4ByteTxRx(portHandler,1, 116, int(goal_position))
                

#         cv2.imshow("Color Image", color_image)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

# finally:
#     print("\n")
#     pipeline.stop()


# import math
# import pyrealsense2 as rs
# import cv2
# import numpy as np
# import time

# from ultralytics import YOLO

# import sys, tty, termios
# fd = sys.stdin.fileno()
# old_settings = termios.tcgetattr(fd)
# def getch():
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return ch

# from dynamixel_sdk import *


# # YOLO 모델 로드
# model = YOLO('yolov8s.pt')

# # RealSense 파이프라인 설정
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.accel)
# config.enable_stream(rs.stream.gyro)

# align_to = rs.stream.color
# align = rs.align(align_to)

# #Dynamixel poter
# portHandler = PortHandler('/dev/ttyUSB0')
# packetHandler = PacketHandler(2.0)

# groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
# groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# portHandler.openPort()
# portHandler.setBaudRate(1000000)

# IDS                   = [1]

# packetHandler.write1ByteTxRx(portHandler, 1, 64, 0)
# packetHandler.write1ByteTxRx(portHandler,1, 11, 3)
# packetHandler.write1ByteTxRx(portHandler,1, 64, 1)

# # 스트리밍 시작
# pipeline.start(config)

# # 새로운 변수 추가
# base_angle = 0
# base_position = 2048

# try:
#     while True:
#         # IMU 데이터 프레임 대기
#         frames = pipeline.wait_for_frames()
#         accel = frames.first_or_default(rs.stream.accel)
        
#         aligned_frames = align.process(frames)
#         color_frame = aligned_frames.get_color_frame()
        
#         color_image = np.asanyarray(color_frame.get_data())

#         results = model.track(source=color_image, persist=True, classes=39, verbose=False)

#         time.sleep(0.005)

#         if accel:
#             accel_data = accel.as_motion_frame().get_motion_data()
#             x, y, z = accel_data.x, accel_data.y, accel_data.z
            
#             # Roll 각도 계산 (라디안)
#             roll = math.atan2(-x, math.sqrt(y*y + z*z))
            
#             # 라디안을 도로 변환
#             roll_degrees = math.degrees(roll)
            
#             # 기준 각도와의 차이 계산
#             angle_diff = roll_degrees - base_angle
            
#             sys.stdout.write(f"\rRoll angle: {roll_degrees:.2f} degrees, Diff: {angle_diff:.2f} degrees")

#             # 각도 차이가 일정 임계값을 넘을 때만 다이나믹셀 움직임
#             if abs(angle_diff) > 5:  # 예: 5도 이상 차이날 때
#                 new_position = base_position - int(angle_diff * (4096/360))
#                 new_position = max(0, min(4095, new_position))  # 값 범위 제한
                
#                 packetHandler.write4ByteTxRx(portHandler, 1, 116, new_position)
                
#                 # 새 위치를 읽어 기준점 업데이트
#                 dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 1, 132)
#                 if dxl_comm_result != COMM_SUCCESS:
#                     print(packetHandler.getTxRxResult(dxl_comm_result))
#                 elif dxl_error != 0:
#                     print(packetHandler.getRxPacketError(dxl_error))
#                 else:
#                     base_position = dxl_present_position
#                     base_angle = roll_degrees

#             # Accelerometer 데이터 읽기
#             accel_data = accel.as_motion_frame().get_motion_data()
#             x, y, z = accel_data.x, accel_data.y, accel_data.z
            
#             # Roll 각도 계산 (라디안)
#             roll = math.atan2(-x, math.sqrt(y*y + z*z))
            
#             # 라디안을 도로 변환
#             roll_degrees = math.degrees(roll)
            
#             # print(f"Roll angle: {roll_degrees:.2f} degrees")
#             sys.stdout.write(f"\rRoll angle: {roll_degrees:.2f} degrees")

#             goal_position = 2048

            
#             if roll_degrees >0:
#                 goal_position -= roll_degrees * (4096/360)
#                 packetHandler.write4ByteTxRx(portHandler,1, 116, int(goal_position))

#             elif roll_degrees <0:
#                 goal_position -= roll_degrees * (4096/360)
#                 packetHandler.write4ByteTxRx(portHandler,1, 116, int(goal_position))
                

#         cv2.imshow("Color Image", color_image)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

# finally:
#     print("\n")
#     pipeline.stop()


import math
import pyrealsense2 as rs
import cv2
import numpy as np
import time
import threading
import sys, tty, termios

from ultralytics import YOLO
from dynamixel_sdk import *

# YOLO 모델 로드
model = YOLO('yolov8s.pt')

# RealSense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)

align_to = rs.stream.color
align = rs.align(align_to)

# Dynamixel 설정
portHandler = PortHandler('/dev/ttyUSB0')
packetHandler = PacketHandler(2.0)

portHandler.openPort()
portHandler.setBaudRate(1000000)

# Dynamixel 초기 설정
packetHandler.write1ByteTxRx(portHandler, 1, 64, 0)
packetHandler.write1ByteTxRx(portHandler, 1, 11, 3)
packetHandler.write1ByteTxRx(portHandler, 1, 64, 1)

def get_roll_angle(accel):
    accel_data = accel.as_motion_frame().get_motion_data()
    x, y, z = accel_data.x, accel_data.y, accel_data.z
    roll = math.atan2(-x, math.sqrt(y*y + z*z))
    return math.degrees(roll)

def get_motor_position(portHandler, packetHandler):
    position, result, error = packetHandler.read4ByteTxRx(portHandler, 1, 132)
    if result != COMM_SUCCESS or error != 0:
        print(f"Failed to get motor position. Result: {result}, Error: {error}")
        return None
    return position

def set_motor_position(portHandler, packetHandler, position):
    result, error = packetHandler.write4ByteTxRx(portHandler, 1, 116, position)
    if result != COMM_SUCCESS or error != 0:
        print(f"Failed to set motor position. Result: {result}, Error: {error}")

def initial_pos(accel, portHandler, packetHandler, frames):
    frames = pipeline.wait_for_frames()
    accel = frames.first_or_default(rs.stream.accel)

    initial_angle = get_roll_angle(accel)
    initial_position = get_motor_position(portHandler, packetHandler)

    return initial_angle, initial_position

def current_pos(accel, portHandler, packetHandler, frames):
    # 1초 후 각도와 모터 위치 측정
    frames = pipeline.wait_for_frames()
    accel = frames.first_or_default(rs.stream.accel)
    
    current_angle = get_roll_angle(accel)
    current_position = get_motor_position(portHandler, packetHandler)

    return current_angle, current_position

def caculate_angle():
    # 각도 변화 계산
    angle_change = current_angle - initial_angle




# 스트리밍 시작
pipeline.start(config)

try:
    while True:
        # 현재 각도와 모터 위치 측정
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        
        initial_angle = get_roll_angle(accel)
        initial_position = get_motor_position(portHandler, packetHandler)

        if initial_position is None:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        # results = model.track(source=color_image, persist=True, classes=39, verbose=False)

        # 1초 대기
        time.sleep(0.01)

        # 1초 후 각도와 모터 위치 측정
        frames = pipeline.wait_for_frames()
        accel = frames.first_or_default(rs.stream.accel)

        current_angle = get_roll_angle(accel)
        current_position = get_motor_position(portHandler, packetHandler)

        if current_position is None:
            continue

        # 각도 변화 계산
        angle_change = current_angle - initial_angle

        # 모터 위치 조정
        new_position = int(current_position - angle_change * (4096/360))
        new_position = max(0, min(4095, new_position))  # 범위 제한

        # 새 위치로 모터 이동
        set_motor_position(portHandler, packetHandler, new_position)

        print(f"Angle change: {angle_change:.2f}, New position: {new_position}")

        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopping...")

finally:
    print("\n")
    pipeline.stop()
    portHandler.closePort()
    cv2.destroyAllWindows()

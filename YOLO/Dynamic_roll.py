
import threading
import math
import pyrealsense2 as rs
import numpy as np
import time
import sys
import cv2
import collections

from pynput import keyboard
from ultralytics import YOLO

from dynamixel_sdk import *

model = YOLO("yolov8s.pt")

# RealSense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

align_to = rs.stream.color
align = rs.align(align_to)

#Dynamixel poter
portHandler = PortHandler('/dev/ttyUSB0')
packetHandler = PacketHandler(2.0)

groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

portHandler.openPort()
portHandler.setBaudRate(1000000)

IDS                   = 1

packetHandler.write1ByteTxRx(portHandler, IDS, 64, 0)
packetHandler.write1ByteTxRx(portHandler,IDS, 11, 3)
packetHandler.write1ByteTxRx(portHandler,IDS, 64, 1)

# 스트리밍 시작
pipeline.start(config)

stop_flag = False

def Profile(vaule_velocity, vaule_accel):
    packetHandler.write4ByteTxRx(portHandler, IDS, 112, vaule_velocity)
    packetHandler.write4ByteTxRx(portHandler, IDS, 108, vaule_accel)

def angle_caculate(x, y, z):
    roll = math.atan2(-x, math.sqrt(y*y + z*z))
    roll_degrees = math.degrees(roll)

    return roll_degrees

def on_press(key):
    global stop_flag
    try:
        if key.char == 'q':  # 'q' 키를 눌렀을 때
            print("\nSTOP")
            stop_flag = True 

    except AttributeError:
        pass       

# 새로운 변수 추가
base_angle = 0
base_position = 2048

roll_buffer = collections.deque(maxlen=3)

############################################# 메인 함수 #################################################

try:

    # 키보드 입력 쓰레드 시작
    keyboard_thread = threading.Thread(target=keyboard.Listener(on_press=on_press).start, daemon=True)
    keyboard_thread.start()

    while stop_flag == False:

        # IMU 데이터 프레임 대기
        frames = pipeline.wait_for_frames()
        accel = frames.first_or_default(rs.stream.accel)

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        color_image = np.asanyarray(color_frame.get_data())

        results = model.track(source=color_image, persist=True, classes=0, verbose=False)

        time.sleep(0.00001)

        if accel:
            accel_data = accel.as_motion_frame().get_motion_data()
            x, y, z = accel_data.x, accel_data.y, accel_data.z
            
            roll_degrees = angle_caculate(x, y, z)

            roll_buffer.append(roll_degrees)
            roll_degrees = sum(roll_buffer) / len(roll_buffer)
            roll_degrees = int(roll_degrees)
            
            #기준각도, roll 각도 출력
            sys.stdout.write(f"\rRoll angle: {roll_degrees:.2f} degrees".ljust(60))
    

            new_position = base_position - int(roll_degrees * (4096/720))

            if -5 < roll_degrees < 5 :
                new_position = base_position

            new_position = max(0, min(4095, new_position))  # 값 범위 제한
            
            packetHandler.write4ByteTxRx(portHandler, IDS, 116, new_position)
            Profile(350, 60)

            # 새 위치를 읽어 기준점 업데이트
            dxl_present_position = packetHandler.read4ByteTxRx(portHandler, IDS, 132)[0]           
            base_position = dxl_present_position


        cv2.imshow("Color Image", results[0].plot())    #results[0]는 source = color_image 부분으로 읽어드린 비디오를 화면에 보여줌
        if cv2.waitKey(1) & 0xFF == ord('q'):           #results[0].orig.img는 비디오의 원본 이미지, results[0].plot()은 객체 표시 및 라벨링한 비디오를 보여줌
            break

finally:
    packetHandler.write4ByteTxRx(portHandler,IDS, 116, 2048)
    time.sleep(1)
    packetHandler.write1ByteTxRx(portHandler,IDS, 64, 0)

    print("\n")
    pipeline.stop()
    
    

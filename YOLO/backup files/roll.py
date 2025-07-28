# import math
# import pyrealsense2 as rs
# import cv2
# import numpy as np
# import time

# from ultralytics import YOLO

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
            
#             print(f"Roll angle: {roll_degrees:.2f} degrees")


# finally:
#     pipeline.stop()


import math
import pyrealsense2 as rs

# RealSense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)

# 스트리밍 시작
pipeline.start(config)

try:
    while True:
        # IMU 데이터 프레임 대기
        frames = pipeline.wait_for_frames()
        accel = frames.first_or_default(rs.stream.accel)
        
        if accel:
            # Accelerometer 데이터 읽기
            accel_data = accel.as_motion_frame().get_motion_data()
            x, y, z = accel_data.x, accel_data.y, accel_data.z
            
            # Roll 각도 계산 (라디안)
            roll = math.atan2(-x, math.sqrt(y*y + z*z))
            
            # 라디안을 도로 변환
            roll_degrees = math.degrees(roll)
            
            print(f"Roll angle: {roll_degrees:.2f} degrees")

finally:
    pipeline.stop()


import pyrealsense2 as rs
import numpy as np
import math

# RealSense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

# 파이프라인 시작
pipeline.start(config)

try:
    while True:
        # 프레임 대기
        frames = pipeline.wait_for_frames()
        
        # 가속도계와 자이로스코프 데이터 가져오기
        accel = frames.first_or_default(rs.stream.accel)
        gyro = frames.first_or_default(rs.stream.gyro)
        
        if accel and gyro:
            # 가속도계 데이터
            ax, ay, az = accel.as_motion_frame().get_motion_data()
            
            # 자이로스코프 데이터
            gx, gy, gz = gyro.as_motion_frame().get_motion_data()
            
            # Roll, Pitch 계산 (가속도계 데이터 사용)
            roll = math.atan2(ay, az) * 180.0 / math.pi
            pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180.0 / math.pi
            
            # Yaw 계산 (자이로스코프 데이터 적분 필요, 여기서는 간단히 표시)
            yaw = gz  # 실제 응용에서는 시간에 따른 적분이 필요합니다
            
            print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

except Exception as e:
    print(e)
finally:
    pipeline.stop()

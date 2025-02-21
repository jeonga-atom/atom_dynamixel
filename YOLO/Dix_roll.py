import threading
import time
import numpy as np
import cv2
import pyrealsense2 as rs

# Realsense 및 모터 핸들러 설정 (가정)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.accel)
pipeline.start(config)

# 모터 관련 (가정)
portHandler = None
packetHandler = None

# 글로벌 변수
initial_angle = 0
initial_position = 0
current_angle = 0
current_position = 0
lock = threading.Lock()  # 스레드 간 데이터 충돌 방지


def get_roll_angle(accel):
    """가속도계를 이용하여 각도를 측정하는 함수 (가정)"""
    return np.arctan2(accel.y, accel.z) * (180 / np.pi)


def get_motor_position(portHandler, packetHandler):
    """모터 위치를 측정하는 함수 (가정)"""
    return np.random.randint(0, 4096)  # 실제 모터에서 위치를 읽는 코드로 변경 필요


def set_motor_position(portHandler, packetHandler, position):
    """모터 위치를 설정하는 함수 (가정)"""
    print(f"모터 위치 변경: {position}")


def get_initial_position():
    """초기 모터 위치와 각도를 측정하는 함수"""
    global initial_angle, initial_position
    while True:
        aligned_frames = pipeline.wait_for_frames()
        accel = aligned_frames.first_or_default(rs.stream.accel)

        with lock:
            initial_angle = get_roll_angle(accel)
            initial_position = get_motor_position(portHandler, packetHandler)

        print(f"[초기 측정] 각도: {initial_angle}, 위치: {initial_position}")
        time.sleep(1)


def get_current_position():
    """현재 모터 위치와 각도를 측정하는 함수"""
    global current_angle, current_position
    while True:
        aligned_frames = pipeline.wait_for_frames()
        accel = aligned_frames.first_or_default(rs.stream.accel)

        with lock:
            current_angle = get_roll_angle(accel)
            current_position = get_motor_position(portHandler, packetHandler)

        print(f"[현재 측정] 각도: {current_angle}, 위치: {current_position}")
        time.sleep(1)


def calculate_position_difference():
    """초기 위치와 현재 위치 차이를 계산하는 함수"""
    while True:
        with lock:
            angle_change = current_angle - initial_angle
            if current_position is not None:
                new_position = int(current_position - angle_change * (4096 / 360))
                new_position = max(0, min(4095, new_position))  # 범위 제한

        print(f"[계산] 각도 변화: {angle_change:.2f}, 새 위치: {new_position}")
        time.sleep(1)


def move_motor():
    """계산된 위치로 모터를 이동하는 함수"""
    while True:
        with lock:
            if current_position is not None:
                set_motor_position(portHandler, packetHandler, current_position)

        time.sleep(1)


# 🔹 스레드 생성 및 실행
thread1 = threading.Thread(target=get_initial_position, daemon=True)
thread2 = threading.Thread(target=get_current_position, daemon=True)
thread3 = threading.Thread(target=calculate_position_difference, daemon=True)
thread4 = threading.Thread(target=move_motor, daemon=True)

thread1.start()
thread2.start()
thread3.start()
thread4.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopping...")

finally:
    print("프로그램 종료")
    pipeline.stop()
    if portHandler:
        portHandler.closePort()
    cv2.destroyAllWindows()

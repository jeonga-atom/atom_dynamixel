import cv2
import numpy as np
import pyrealsense2 as rs
import time
import sys

from ultralytics import YOLO
from dynamixel_sdk import *

# YOLO 모델 로드
model = YOLO('yolov8s.pt')

# 다이나믹셀 설정
portHandler = PortHandler('/dev/ttyUSB0')
packetHandler = PacketHandler(2.0)

groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

portHandler.openPort()
portHandler.setBaudRate(1000000)

# 리얼센스 카메라 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# 이미지 필터
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()

# 카메라 매트릭스와 왜곡 계수
camera_matrix = np.array([[623.54013868, 0, 331.5450823],
                          [0, 626.04451649, 246.27759741],
                          [0, 0, 1]])
dist_coeffs = np.array([0.11563788, -0.00684786, -0.00223002, 0.00458697, -0.52293788])

#다이나믹셀 기본 값 설정
IDS_pos                 = 1              # 조향각

# 캘리브레이션
def undistort_image(image):
    h, w = image.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted

def Profile(vaule_velocity, vaule_accel):
    packetHandler.write4ByteTxRx(portHandler, IDS_pos, 112, vaule_velocity)
    packetHandler.write4ByteTxRx(portHandler, IDS_pos, 108, vaule_accel)

packetHandler.write1ByteTxRx(portHandler, 1, 64, 0)
packetHandler.write1ByteTxRx(portHandler,1, 11, 3)
packetHandler.write1ByteTxRx(portHandler,1, 64, 1)

goal_pos = 2048

# 화면 세로 중심 라인 설정
vertical_center_line = 240

#메인 함수
try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        color_image = np.asanyarray(color_frame.get_data())

        color_image = undistort_image(color_image)

        results = model.track(source=color_image, persist=True, classes=67, verbose=False)

        time.sleep(0.005)

        # 터미널 출력할 정보 변수 초기화
        output_text = ""
        offset_y = ""

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = box.conf[0].cpu().numpy()
                class_id = box.cls[0].cpu().numpy()

                # 바운딩 박스 중심 계산
                pixel_x = (x1 + x2) // 2
                pixel_y = y2 - 15

                cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                cv2.circle(color_image, (pixel_x, (y1 + y2) // 2), 2, (0, 0, 255), 2, cv2.LINE_AA)

                # 객체 중심이 화면 세로 중심 라인과 얼마나 떨어져 있는지 계산
                offset_y = vertical_center_line - ((y1 + y2) // 2)

                # 오프셋에 따라 짐벌 각도 조절 (임시)
                # 짐벌 각도를 직접 설정하는 대신, 오프셋에 따라 목표 위치를 조절
                # 각도 오프셋을 위치 변화량으로 변환 (임시 변환 계수 사용)
                position_offset = int(offset_y * 1)  # 1은 임시 변환 계수, 조절 필요

                goal_pos = 2048  # 현재 위치를 목표 위치로 설정
                goal_pos += position_offset  # 오프셋을 더해 목표 위치 조정

                # 목표 위치가 다이나믹셀의 범위를 벗어나지 않도록 제한
                goal_pos = max(0, min(4095, goal_pos))

                # 다이나믹셀에 목표 위치 쓰기
                packetHandler.write4ByteTxRx(portHandler, 1, 116, int(goal_pos))
                Profile(350, 100)
        

        # 터미널 출력 (덮어쓰기)
        sys.stdout.write(f"\rOffset Y: {offset_y}".ljust(80))  # 길이 고정하여 이전 값 삭제 방지
        sys.stdout.flush()

        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        
finally:
    print("\n")
    pipeline.stop()
    cv2.destroyAllWindows()


import os
import cv2
import numpy as np
import pyrealsense2 as rs
import time
import sys
import math

from ultralytics import YOLO
from dynamixel_sdk import *

# 다이나믹셀 키 받아오는 함수
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

#좌표 값 계산 (x,y,z)
def compute_object_position(object_depth, pixel_x, image_width=640, fov_x=69, camera_height=0.14):
    max_angle_x = fov_x / 2
    angle_x = ((pixel_x / (image_width / 2)) - 1) * max_angle_x
    angle_x_rad = np.radians(angle_x)

    d = np.sqrt(object_depth**2 - 0.065**2 + (0.065*np.cos(np.pi/2 - angle_x_rad))**2) + 0.065*np.cos(np.pi/2 - angle_x_rad)

    x = d * np.sin(angle_x_rad)
    z = camera_height
    y = np.sqrt(d**2 * np.cos(angle_x_rad)**2 - camera_height**2)

    return x, y, z

def calculate_gimbal_angles(pixel_x, pixel_y, image_width=640, image_height=480):
    global diff_x, diff_y

    center_x = image_width // 2  # 320
    center_y = image_height // 2  # 240

    diff_x = pixel_x - center_x
    diff_y = ((pixel_y + 15 + y1) // 2)- center_y

    angle_x = 0
    angle_y = 0

    # X축 (수평) 각도 계산
    if abs(diff_x) <= 40:
        angle_x = 0
    elif abs(diff_x) <= 120:
        angle_x = 8.625 * (1 if diff_x > 0 else -1)
    elif abs(diff_x) <= 200:
        angle_x = 17.25 * (1 if diff_x > 0 else -1)
    elif abs(diff_x) <= 280:
        angle_x = 25.875 * (1 if diff_x > 0 else -1)
    else:
        angle_x = 34.5 * (1 if diff_x > 0 else -1)

    # Y축 (수직) 각도 계산
    if abs(diff_y) <= 40:
        angle_y = 0
    elif abs(diff_y) <= 120:
        angle_y = -7 * (1 if diff_y > 0 else -1)
    elif abs(diff_y) <= 200:
        angle_y = -14 * (1 if diff_y > 0 else -1)
    else:
        angle_y = -21 * (1 if diff_y > 0 else -1)

    return angle_x, angle_y

# 초록색 그리드 선 그리기
# def draw_grid_with_angles(image):
#     h, w = image.shape[:2]
    
#     # 가로 선 그리기
#     horizontal_divisions = [40, 120, 200, 280, 360, 440, 520, 600]
#     for y in horizontal_divisions:
#         cv2.line(image, (0, y), (w, y), (0, 255, 0), 1)  # 초록색

#     # 세로 선 그리기
#     vertical_divisions = [40, 120, 200, 280, 360, 440, 520, 600]
#     for x in vertical_divisions:
#         cv2.line(image, (x, 0), (x, h), (0, 255, 0), 1)  # 초록색

    # 빨간색 중심선 그리기
    # center_x = w // 2
    # center_y = h // 2
    # cv2.line(image, (center_x, 0), (center_x, h), (0, 0, 255), 2)  # 빨간색, 두께 2
    # cv2.line(image, (0, center_y), (w, center_y), (0, 0, 255), 2)  # 빨간색, 두께 2

packetHandler.write1ByteTxRx(portHandler, 1, 64, 0)
packetHandler.write1ByteTxRx(portHandler,1, 11, 3)
packetHandler.write1ByteTxRx(portHandler,1, 64, 1)

goal_pos = 2048
diff_angle = (7/360)*4096

# 화면 세로 중심 라인 설정
vertical_center_line = 240

#메인 함수
try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        aligned_depth_frame = aligned_frames.get_depth_frame()
        depth_frame = aligned_depth_frame

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(aligned_depth_frame.get_data())

        color_image = undistort_image(color_image)

        # 그리드 및 중심선 그리기
        # draw_grid_with_angles(color_image)

        results = model.track(source=color_image, persist=True, classes=41, verbose=False)

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

                depth = depth_frame.get_distance(pixel_x, pixel_y)

                obj_x, obj_y, obj_z = compute_object_position(depth, pixel_x, pixel_y)
                gimbal_angle_x, gimbal_angle_y = calculate_gimbal_angles(pixel_x, pixel_y)

                depth_object_name = f"{model.names[int(class_id)]}, depth: {depth:.3f}m"
                position_label = f"({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})"
                gimbal_label = f"Gimbal angles: ({gimbal_angle_x:.2f}, {gimbal_angle_y:.2f})"

                cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                cv2.circle(color_image, (pixel_x, pixel_y), 2, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.circle(color_image, (pixel_x, (y1 + y2) // 2), 2, (0, 0, 255), 2, cv2.LINE_AA)


                cv2.putText(color_image, depth_object_name, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)
                cv2.putText(color_image, position_label, (x1, y1 - 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(color_image, gimbal_label, (x1, y1 - 50), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)

                output_text = f"{depth_object_name}, {position_label}, {gimbal_label}"

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
        

        # 터미널 출력 (덮어쓰기)
        sys.stdout.write(f"\r{output_text.ljust(80)} Offset Y: {offset_y}".ljust(120))  # 길이 고정하여 이전 값 삭제 방지
        sys.stdout.flush()

        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        
finally:
    print("\n")
    pipeline.stop()
    cv2.destroyAllWindows()


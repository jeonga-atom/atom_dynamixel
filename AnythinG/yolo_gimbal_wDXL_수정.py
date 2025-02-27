import cv2
import numpy as np
import pyrealsense2 as rs
import time
import sys

from ultralytics import YOLO

from Dynmxl_Setup_Constant import *
from Dynmxl_initializing import *
from Dynmxl_running import Dynmxl_process_for_ATOM

import math

# YOLO 모델 로드
model = YOLO('yolov8s.pt')

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
    diff_y = ((pixel_y + 15 + y1) // 2) - center_y

    angle_x = 0
    angle_y = 0

    # X축 (수평) 각도 계산
    angle_x = 34.5 * (diff_x / center_x)

    # Y축 (수직) 각도 계산
    # angle_y 계산 식 적어주길 바라... 안 쓰면 angle_y 부분 다 지우면 됨

    return angle_x, angle_y


##### 메인 함수 내 임시 변수 #####
temp_Angle = FORWARD_POS
temp_Velo = 0
max_deg_value = round(MAXIMUM_STEERING_ANG * (4096 / 360))
# Turning_Step = 0

gimbal_angle_x = 0.0

#메인 함수
try:
    while True:
        depth = 0.0
        
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        color_image = undistort_image(color_image)

        results = model.track(source=color_image, persist=True, classes=39, verbose=False)

        time.sleep(0.005)

        # 터미널 출력할 정보 변수 초기화
        output_text = ""

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = box.conf[0].cpu().numpy()
                class_id = box.cls[0].cpu().numpy()

                pixel_x = (x1 + x2) // 2
                pixel_y = y2 - 15

                depth = depth_frame.get_distance(pixel_x, pixel_y)

                if depth > 0:  # 유효한 깊이 값만 처리 -> 이 부분이 들어가면  d = np.sqrt(object_depth**2 - 0.065**2 + (0.065*np.cos(np.pi/2 - angle_x_rad))**2) + 0.065*np.cos(np.pi/2 - angle_x_rad) 터미널에 이렇게 뜨는거 사라짐
                    obj_x, obj_y, obj_z = compute_object_position(depth, pixel_x, pixel_y)
                    gimbal_angle_x, gimbal_angle_y = calculate_gimbal_angles(pixel_x, pixel_y)

                    depth_object_name = f"{model.names[int(class_id)]}, depth: {depth:.3f}m"
                    position_label = f"({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})"
                    gimbal_label = f"Gimbal angles: ({gimbal_angle_x:.2f}, {gimbal_angle_y:.2f})" # 여기 gimbal_angle_y 이 부분도

                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                    cv2.circle(color_image, (pixel_x, pixel_y), 2, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.circle(color_image, (pixel_x, (y1 + y2) // 2), 2, (0, 0, 255), 2, cv2.LINE_AA)


                    cv2.putText(color_image, depth_object_name, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)
                    cv2.putText(color_image, position_label, (x1, y1 - 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (127, 63, 216), 1)
                    cv2.putText(color_image, gimbal_label, (x1, y1 - 50), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 63, 216), 1)

                    output_text = f"{depth_object_name}, {position_label}, {gimbal_label}"


        # 터미널 출력 (덮어쓰기)
        sys.stdout.write(f"\r{output_text.ljust(100)}")  # 길이 고정하여 이전 값 삭제 방지
        sys.stdout.flush()

        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


        if type(depth) == float and (not(math.isnan(depth)) and depth > 0.0):
            if 0.3 <= depth <= 1.1:
                temp_Velo = 200 * math.sqrt((depth - 0.3) / 0.8)
            elif 0.0 < depth < 0.3:
                # temp_Velo = -200 * math.sqrt((0.3 - depth) / 0.5) # 후진 적용 시
                temp_Velo = 0
            elif depth > 1.1:
                temp_Velo = 0
            else:
                temp_Velo = 200
            # print(type(depth))
        else: # Zero or Not a Number
            temp_Velo = 0
            # print(math.isnan(depth))
        
        
        if type(gimbal_angle_x) == float or int:
            temp_Angle = FORWARD_POS + (max_deg_value * (gimbal_angle_x / 34.5))
            '''
            if round(gimbal_angle_x) == 0.0:
                Turning_Step = 0
            else:
                Turning_Step = int((gimbal_angle_x / abs(gimbal_angle_x)) * (abs(gimbal_angle_x) // 8.6))
            
            if Turning_Step == 0:
                temp_Angle = FORWARD_POS
            else:
                temp_Angle = FORWARD_POS + (Turning_Step * max_deg_value)
            '''
        else:
            pass

        # print(f"{depth:0.3f}, {gimbal_angle_x:6.2f}, {temp_Angle:2}")
        
        Dynmxl_process_for_ATOM(Outer_Wheel_Angle_in4096=temp_Angle, Outer_Wheel_Velocity_in200=temp_Velo)


finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    Dynmxl_process_for_ATOM(Outer_Wheel_Angle_in4096=FORWARD_POS, Outer_Wheel_Velocity_in200=0)




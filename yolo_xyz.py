import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import time

# YOLO 임포트
model = YOLO('yolov8s.pt')

# 리얼센스 초기 설정 과정 (파이프라인 ,config)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# cm에서 m 변환
depth_scale = 0.0010000000474974513

# 이미지 필터
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()

camera_matrix = np.array([[623.54013868, 0, 331.5450823],
                          [0, 626.04451649, 246.27759741],
                          [0, 0, 1]])
dist_coeffs = np.array([ 0.11563788, -0.00684786, -0.00223002,  0.00458697, -0.52293788])  

# 왜곡 보정 함수
# def undistort_point(x, y, camera_matrix, dist_coeffs):
#     points = np.array([[x, y]], dtype=np.float32)
#     undistorted_points = cv2.undistortPoints(points, camera_matrix, dist_coeffs, None, camera_matrix)
#     return undistorted_points[0][0]

def undistort_image(image):
    h, w = image.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted


#카메라 좌표 변환 함수
def compute_object_position(object_depth, pixel_x, pixel_y, image_width=640, fov_x=69, camera_height=0.14):
    """
    픽셀 좌표 (x, y) 및 깊이값을 받아서 3D 좌표 (x, y, z)로 변환하는 함수
    """
    max_angle_x = fov_x / 2  # 최대 시야각 (좌우 34.7도)
    angle_x = ((pixel_x/ (image_width / 2)) -1) * max_angle_x
    angle_x_rad = np.radians(angle_x)

    d = np.sqrt(object_depth**2 -0.065**2 +(0.065*np.cos(np.pi/2 -angle_x_rad))**2) +0.065*np.cos(np.pi/2 -angle_x_rad) #depth센서로부터 물체의 거리를 RGB센서 기준으로 물체와릐 거리 계산

    x = d * np.sin(angle_x_rad)  # X 좌표
    z = camera_height  # Z 좌표 (우선 고정)
    y = np.sqrt(d**2 * np.cos(angle_x_rad)**2 - camera_height**2)  # Y 좌표

    # print(angle_x, np.cos(angle_x_rad), np.sin(angle_x_rad))
    return x, y, z

# 동작 시작
try:
    while True:
        # 색깔 프레임, 거리 프레임 얻어오기
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        aligned_depth_frame = aligned_frames.get_depth_frame()
        depth_frame = aligned_frames.get_depth_frame()

        # 수치 행렬화
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(aligned_depth_frame.get_data())

        color_image = undistort_image(color_image)

        # YOLO 감지 수행
        results = model.track(source=color_image, persist=True, classes=67, verbose=False)

        time.sleep(0.005)

        # 본격적인 영상 처리 작업
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = box.conf[0].cpu().numpy()
                class_id = box.cls[0].cpu().numpy()

                # 물체의 중앙 좌표 (x 픽셀, y 픽셀)
                pixel_x = (x1 + x2) // 2
                pixel_y = y2 - 15  # y2 기준

                # pixel_x, pixel_y = undistort_point(pixel_x, pixel_y, camera_matrix, dist_coeffs)
                depth = depth_frame.get_distance(pixel_x, pixel_y) 

                # 3D 좌표 변환 적용
                obj_x, obj_y, obj_z = compute_object_position(depth, pixel_x, pixel_y)
                # 표시할 라벨 생성
                depth_object_name = f"{model.names[int(class_id)]}, depth: {depth:.3f}m"
                position_label = f"({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})"

                # 사각형 처리
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                cv2.circle(color_image, (pixel_x, pixel_y), 2, (0, 255, 0), 2, cv2.LINE_AA)   #초록색으로 점 표시
                #점 찍는 법: cv2.circle(image, center, radius, color, thickness)

                # 텍스트 기입 (객체 이름 + 거리 + 3D 좌표)
                cv2.putText(color_image, depth_object_name, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)
                cv2.putText(color_image, position_label, (x1, y1 - 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)

                print(f"({depth_object_name}, {position_label} ")

        # 이미지 디스플레이
        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
# 중단
finally:
    pipeline.stop()
    cv2.destroyAllWindows()

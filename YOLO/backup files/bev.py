
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import time

# YOLO 모델 로드
model = YOLO('yolov8s.pt')

# 리얼센스 초기화
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)   # 깊이 프레임을 컬러 프레임과 정렬하는 객체 생성

# 카메라 매트릭스 및 왜곡 계수
camera_matrix = np.array([[623.54, 0, 331.54],
                          [0, 626.04, 246.28],
                          [0, 0, 1]])   # 카메라 행렬(초점 거리 및 광학 중심)
dist_coeffs = np.array([0.1156, -0.0068, -0.0022, 0.0046, -0.5229])   # 카메라 렌즈의 왜곡 계수(왜곡 보정을 위한 계수)

# 왜곡 보정
def undistort_image(image):
    h, w = image.shape[:2]  # 이미지의 높이(h)와 너비(w) 추출
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h)) # 왜곡 보정을 위한 새로운 카메라 행렬 계산
    return cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)    # 왜곡이 제거된 이미지를 반환

# 픽셀 좌표 → 실제 3D 좌표 변환

def compute_object_position(object_depth, pixel_x, pixel_y, image_width=640, fov_x=69, camera_height=0.14):
    max_angle_x = fov_x / 2
    angle_x = ((pixel_x / (image_width / 2)) - 1) * max_angle_x
    angle_x_rad = np.radians(angle_x)

    # NaN 체크 및 예외 처리 추가
    if object_depth <= 0 or np.isnan(object_depth):
        return None, None, None

    d = np.sqrt(max(0, object_depth**2 - 0.065**2 + (0.065*np.cos(np.pi/2 - angle_x_rad))**2)) + 0.065*np.cos(np.pi/2 - angle_x_rad)

    x = d * np.sin(angle_x_rad)
    z = camera_height
    y = np.sqrt(max(0, d**2 * np.cos(angle_x_rad)**2 - camera_height**2))

    return x, y, z


# 탑뷰 이미지 크기 설정, 탑뷰(BEV)에서 실제 거리를 픽셀 단위로 변환하는 비율을 설정
BEV_WIDTH = 500  # 너비
BEV_HEIGHT = 500  # 높이
SCALE = 100  # 1m = 100px 변환 비율

# 메인 루프
try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue  # 컬러 또는 깊이 데이터가 없으면 루프를 다시 실행

        color_image = np.asanyarray(color_frame.get_data()) # 컬레 프레임 Numpy 배열로 변환
        color_image = undistort_image(color_image)  # 왜곡 보정 적용
        
        results = model.track(source=color_image, persist=True, classes=41, verbose=False)

        time.sleep(0.005)

        # Bird's Eye View (탑뷰) 초기화
        bev_image = np.zeros((BEV_HEIGHT, BEV_WIDTH, 3), dtype=np.uint8)  # 검은 배경
        cv2.line(bev_image, (BEV_WIDTH // 2, 0), (BEV_WIDTH // 2, BEV_HEIGHT), (255, 255, 255), 2)  # 중앙선

        for result in results:
            if not result.boxes:
                continue

            boxes = result.boxes.xyxy.cpu().numpy().astype(int)
            confidences = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            #.cpu().numpy().astype(int): 데이터를 numpy 배열로 변환하고 정수 타입으로 변환(이 과정은 모델이 GPU에서 연산하는 경우 결과를 CPU로 가져오기 위해 필요)

            for (x1, y1, x2, y2), conf, class_id in zip(boxes, confidences, class_ids):
                pixel_x = (x1 + x2) // 2
                pixel_y = y2 - 15  
                depth = depth_frame.get_distance(pixel_x, pixel_y)

                obj_x, obj_y, obj_z = compute_object_position(depth, pixel_x, pixel_y)

                if obj_x is None or obj_y is None or obj_z is None:
                    continue  # 잘못된 값이 계산되면 이 객체를 건너뜁니다.

                # Bird's Eye View 좌표 변환
                bev_x = int(BEV_WIDTH // 2 + obj_x * SCALE) # bev_x: 중심선(BEV_WIDTH // 2) 기준으로 obj_x만큼 이동 (SCALE 적용)
                bev_y = int(BEV_HEIGHT - obj_y * SCALE) # bev_y: 아래쪽을 원점(0)으로 하여 obj_y만큼 위로 이동

                # 범위 제한 (이미지 크기를 넘지 않도록)
                bev_x = np.clip(bev_x, 0, BEV_WIDTH)    #np.clip(arr, min, max) → 배열 값들을 min~max 범위로 제한
                bev_y = np.clip(bev_y, 0, BEV_HEIGHT)

                # 탑뷰에 객체 표시
                cv2.circle(bev_image, (bev_x, bev_y), 5, (0, 255, 0), -1)
                cv2.putText(bev_image, f"{model.names[class_id]}", (bev_x - 10, bev_y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # 카메라 화면에도 정보 표시
                depth_info = f"{model.names[class_id]}, depth: {depth:.3f}m"
                position_info = f"({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})"

                cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                cv2.circle(color_image, (pixel_x, pixel_y), 2, (0, 255, 0), 2, cv2.LINE_AA)

                cv2.putText(color_image, depth_info, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)
                cv2.putText(color_image, position_info, (x1, y1 - 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)

                print(f"{depth_info}, {position_info}")

        # 영상 출력
        cv2.imshow("Color Image", color_image)
        cv2.imshow("Bird's Eye View", bev_image)  # BEV 출력
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

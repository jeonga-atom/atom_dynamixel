import cv2
import numpy as np
import pyrealsense2 as rs
import open3d as o3d
from ultralytics import YOLO

# 1️⃣ RealSense 카메라 초기화
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# 카메라 Intrinsics 가져오기
depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

# 2️⃣ YOLO 모델 로드
model = YOLO("yolov8n.pt")

def pixel_to_3d(x, y, depth_value):
    """ 2D 픽셀 좌표를 3D 공간 좌표로 변환 """
    X = (x - depth_intrinsics.ppx) * depth_value / depth_intrinsics.fx
    Y = (y - depth_intrinsics.ppy) * depth_value / depth_intrinsics.fy
    Z = depth_value
    return [X, Y, Z]

while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())
    
    # 3️⃣ YOLO 객체 감지
    results = model(color_image)

    geometries = []  # Open3D 시각화 객체 리스트

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # BBox 좌표
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 중심점

            # Depth 값 추출
            depth_value = depth_frame.get_distance(cx, cy)

            # 4️⃣ 3D 바운딩 박스 좌표 변환
            p1 = pixel_to_3d(x1, y1, depth_value)
            p2 = pixel_to_3d(x2, y1, depth_value)
            p3 = pixel_to_3d(x2, y2, depth_value)
            p4 = pixel_to_3d(x1, y2, depth_value)

            # 3D 박스 생성
            points = np.array([p1, p2, p3, p4])  # 4개의 꼭짓점
            lines = [[0, 1], [1, 2], [2, 3], [3, 0]]  # 바운딩 박스 연결선
            colors = [[1, 0, 0] for _ in range(len(lines))]  # 빨간색

            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(points)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector(colors)

            geometries.append(line_set)

    # 5️⃣ Open3D 시각화 실행
    if geometries:
        o3d.visualization.draw_geometries(geometries)

    cv2.imshow("RGB Image", color_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()

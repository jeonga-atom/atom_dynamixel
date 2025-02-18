
# import cv2
# import numpy as np
# import pyrealsense2 as rs
# from ultralytics import YOLO
# import time

# # YOLO 모델 로드
# model = YOLO('yolov8s.pt')

# # 리얼센스 카메라 설정
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# pipeline.start(config)

# align_to = rs.stream.color
# align = rs.align(align_to)

# # 이미지 필터
# spatial = rs.spatial_filter()
# temporal = rs.temporal_filter()

# # 카메라 매트릭스와 왜곡 계수
# camera_matrix = np.array([[623.54013868, 0, 331.5450823],
#                           [0, 626.04451649, 246.27759741],
#                           [0, 0, 1]])
# dist_coeffs = np.array([0.11563788, -0.00684786, -0.00223002, 0.00458697, -0.52293788])

# # 카메라 왜곡 보정 함수
# def undistort_image(image):
#     h, w = image.shape[:2]
#     new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
#     undistorted = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)
#     return undistorted

# # 물체의 위치(x,y,z) 반환하는 함수
# def compute_object_position(object_depth, pixel_x, pixel_y, image_width=640, fov_x=69, camera_height=0.14):
#     max_angle_x = fov_x / 2
#     angle_x = ((pixel_x / (image_width / 2)) - 1) * max_angle_x
#     angle_x_rad = np.radians(angle_x)

#     d = np.sqrt(object_depth**2 - 0.065**2 + (0.065*np.cos(np.pi/2 - angle_x_rad))**2) + 0.065*np.cos(np.pi/2 - angle_x_rad)

#     x = d * np.sin(angle_x_rad)
#     z = camera_height
#     y = np.sqrt(d**2 * np.cos(angle_x_rad)**2 - camera_height**2)

#     return x, y, z

# # 각도 반환하는 함수
# def calculate_gimbal_angles(pixel_x, pixel_y, image_width=640, image_height=480):
#     cell_width = 80
#     cell_height = 80

#     center_x = image_width // 2     #320
#     center_y = image_height // 2    #240

#     diff_x = pixel_x - center_x
#     diff_y = pixel_y - center_y

#     angle_x = 0
#     angle_y = 0

#     if abs(diff_x) > cell_width:
#         angle_x = (diff_x // cell_width) * 8.625

#     if abs(diff_y) > cell_height:
#         angle_y = (diff_y // cell_height) * 7

#     max_angle_y = 21
#     angle_y = max(min(angle_y, max_angle_y), -max_angle_y)

#     return angle_x, angle_y

# # 화면에 격자 그리는 함수
# def draw_grid(image, rows=6, cols=8):
#     h, w = image.shape[:2]
#     dy, dx = h // rows, w // cols

#     for x in range(dx, w, dx):
#         cv2.line(image, (x, 0), (x, h), (255, 255, 255), 1)

#     for y in range(dy, h, dy):
#         cv2.line(image, (0, y), (w, y), (255, 255, 255), 1)

# try:
#     while True:
#         frames = pipeline.wait_for_frames()
#         aligned_frames = align.process(frames)
#         color_frame = aligned_frames.get_color_frame()
#         aligned_depth_frame = aligned_frames.get_depth_frame()
#         depth_frame = aligned_frames.get_depth_frame()

#         color_image = np.asanyarray(color_frame.get_data())
#         depth_image = np.asanyarray(aligned_depth_frame.get_data())

#         color_image = undistort_image(color_image)

#         # 그리드 그리기
#         draw_grid(color_image)

#         results = model.track(source=color_image, persist=True, classes=39, verbose=False)

#         time.sleep(0.005)

#         for result in results:
#             boxes = result.boxes
#             for box in boxes:
#                 x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
#                 confidence = box.conf[0].cpu().numpy()
#                 class_id = box.cls[0].cpu().numpy()

#                 pixel_x = (x1 + x2) // 2
#                 pixel_y = y2 - 15

#                 depth = depth_frame.get_distance(pixel_x, pixel_y)

#                 obj_x, obj_y, obj_z = compute_object_position(depth, pixel_x, pixel_y)
#                 gimbal_angle_x, gimbal_angle_y = calculate_gimbal_angles(pixel_x, pixel_y)

#                 depth_object_name = f"{model.names[int(class_id)]}, depth: {depth:.3f}m"
#                 position_label = f"({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})"
#                 gimbal_label = f"Gimbal angles: ({gimbal_angle_x:.2f}, {gimbal_angle_y:.2f})"

#                 cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
#                 cv2.circle(color_image, (pixel_x, pixel_y), 2, (0, 255, 0), 2, cv2.LINE_AA)

#                 cv2.putText(color_image, depth_object_name, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)
#                 cv2.putText(color_image, position_label, (x1, y1 - 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 1)
#                 cv2.putText(color_image, gimbal_label, (x1, y1 - 50), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 1)

#                 print(f"{depth_object_name}, {position_label}, {gimbal_label}")

#         cv2.imshow("Color Image", color_image)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

# finally:
#     pipeline.stop()
#     cv2.destroyAllWindows()


import cv2
import numpy as np
import pyrealsense2 as rs
import time

from ultralytics import YOLO

# YOLO 모델 로드
model = YOLO('yolov8n.pt')

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

def undistort_image(image):
    h, w = image.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted

def compute_object_position(object_depth, pixel_x, pixel_y, image_width=640, fov_x=69, camera_height=0.14):
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

    cell_width = 80
    cell_height = 80

    center_x = image_width // 2     #320
    center_y = image_height // 2    #240

    diff_x = pixel_x - center_x
    diff_y = ((pixel_y + 15 + y1) // 2) - center_y

    angle_x = 0
    angle_y = 0

    if abs(diff_x) > cell_width:
        angle_x = (diff_x // cell_width) * 8.625

    if abs(diff_y) > cell_height:
        angle_y = (diff_y // cell_height) * -7

    max_angle_y = 21
    angle_y = max(min(angle_y, max_angle_y), -max_angle_y)

    return angle_x, angle_y

def draw_grid_with_angles(image, rows=5, cols=7):
    h, w = image.shape[:2]
    
    dy, dx = h // rows, w // cols

    # 초록색 그리드 선 그리기
    for x in range(dx, w, dx):
        cv2.line(image, (x, 0), (x, h), (0, 255, 0), 1)  # 초록색

    for y in range(dy, h, dy):
        cv2.line(image, (0, y), (w, y), (0, 255, 0), 1)  # 초록색

    # 빨간색 중심선 그리기
    center_x = w // 2
    center_y = h // 2
    cv2.line(image, (center_x, 0), (center_x, h), (0, 0, 255), 2)  # 빨간색, 두께 2
    cv2.line(image, (0, center_y), (w, center_y), (0, 0, 255), 2)  # 빨간색, 두께 2

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        aligned_depth_frame = aligned_frames.get_depth_frame()
        depth_frame = aligned_frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(aligned_depth_frame.get_data())

        color_image = undistort_image(color_image)

        # 그리드 및 중심선 그리기
        draw_grid_with_angles(color_image)

        results = model.track(source=color_image, persist=True, classes=39, verbose=False)

        time.sleep(0.005)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = box.conf[0].cpu().numpy()
                class_id = box.cls[0].cpu().numpy()

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

                print(f"{depth_object_name}, {position_label}, {gimbal_label}")

        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()




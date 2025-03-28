import cv2
import numpy as np
import pyrealsense2 as rs
import time
import sys
import rclpy
import math

from ultralytics import YOLO
from any_pkg.Dynmxl_Setup_Constant import *
from any_pkg.Dynmxl_initializing import *
from any_pkg.Dynmxl_running import Dynmxl_process_for_ATOM
from rclpy.node import Node
from std_msgs.msg import String

camera_matrix = np.array([[623.54013868, 0, 331.5450823], [0, 626.04451649, 246.27759741], [0, 0, 1]])
dist_coeffs = np.array([0.11563788, -0.00684786, -0.00223002, 0.00458697, -0.52293788])

class atom_pub(Node):
    def __init__(self):
        super().__init__('atompub') # 노드 이름 설정
        self.publisher = self.create_publisher(String, 'acc_topic', 10) # 퍼블리셔 생성
        self.timer = self.create_timer(1.0, self.pub_message)  # 1초마다 실행되는 타이머 설정
        self.pipeline = rs.pipeline() # 리얼센스 카메라 설정
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        self.align_to = rs.stream.color # 이미지 필터
        self.align = rs.align(self.align_to)
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.model = YOLO('yolov8s.pt') # YOLO 모델 로드
        self.temp_Angle = FORWARD_POS  # 메인 함수 내 임시 변수
        self.temp_Velo = 0
        self.max_deg_value = round(MAXIMUM_STEERING_ANG * (4096 / 360))
        # Turning_Step = 0

        self.gimbal_angle_x = 0.0

    # 캘리브레이션
    def undistort_image(self, image):
        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
        undistorted = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)
        return undistorted
    
    #좌표 값 계산 (x,y,z)
    def compute_object_position(self, object_depth, pixel_x, image_width=640, fov_x=69, camera_height=0.14):
        max_angle_x = fov_x / 2
        angle_x = ((pixel_x / (image_width / 2)) - 1) * max_angle_x
        angle_x_rad = np.radians(angle_x)

        d = np.sqrt(object_depth**2 - 0.065**2 + (0.065*np.cos(np.pi/2 - angle_x_rad))**2) + 0.065*np.cos(np.pi/2 - angle_x_rad)

        x = d * np.sin(angle_x_rad)
        z = camera_height
        y = np.sqrt(d**2 * np.cos(angle_x_rad)**2 - camera_height**2)

        return x, y, z
    
    def calculate_gimbal_angles(self, pixel_x, pixel_y, image_width=640, image_height=480):
        global diff_x, diff_y

        center_x = image_width // 2  # 320
        center_y = image_height // 2  # 240

        diff_x = pixel_x - center_x
        diff_y = pixel_y - center_y

        angle_x = 0
        angle_y = 0

        # X축 (수평) 각도 계산
        angle_x = 34.5 * (diff_x / center_x)

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
    def draw_grid_with_angles(self, image):
        h, w = image.shape[:2]
        
        # 가로 선 그리기
        horizontal_divisions = [40, 120, 200, 280, 360, 440, 520, 600]
        for y in horizontal_divisions:
            cv2.line(image, (0, y), (w, y), (0, 255, 0), 1)  # 초록색

        # 세로 선 그리기
        vertical_divisions = [40, 120, 200, 280, 360, 440, 520, 600]
        for x in vertical_divisions:
            cv2.line(image, (x, 0), (x, h), (0, 255, 0), 1)  # 초록색

        # 빨간색 중심선 그리기
        center_x = w // 2
        center_y = h // 2
        cv2.line(image, (center_x, 0), (center_x, h), (0, 0, 255), 2)  # 빨간색, 두께 2
        cv2.line(image, (0, center_y), (w, center_y), (0, 0, 255), 2)  # 빨간색, 두께 2


    def pub_message(self):
        try:
            while True:
                msg = String()
                depth = 0.0
                
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                aligned_depth_frame = aligned_frames.get_depth_frame()
                depth_frame = aligned_frames.get_depth_frame()

                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(aligned_depth_frame.get_data())

                color_image = self.undistort_image(color_image)

                # 그리드 및 중심선 그리기
                self.draw_grid_with_angles(color_image)

                results = self.model.track(source=color_image, persist=True, classes=39, verbose=False)

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
                        pixel_y = (y1 + y2) // 2 #y2 - 15
                        pixel_yy = y2 - 15

                        depth = depth_frame.get_distance(pixel_x, pixel_y)

                        obj_x, obj_y, obj_z = self.compute_object_position(depth, pixel_x, pixel_yy)
                        self.gimbal_angle_x, gimbal_angle_y = self.calculate_gimbal_angles(pixel_x, pixel_y)

                        depth_object_name = f"{self.model.names[int(class_id)]}, depth: {depth:.3f}m"
                        position_label = f"({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})"
                        gimbal_label = f"Gimbal angles: ({self.gimbal_angle_x:.2f}, {gimbal_angle_y:.2f})"

                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                        cv2.circle(color_image, (pixel_x, pixel_yy), 2, (0, 255, 0), 2, cv2.LINE_AA)
                        cv2.circle(color_image, (pixel_x, pixel_y), 2, (0, 0, 255), 2, cv2.LINE_AA)


                        cv2.putText(color_image, depth_object_name, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)
                        cv2.putText(color_image, position_label, (x1, y1 - 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (127, 63, 216), 1)
                        cv2.putText(color_image, gimbal_label, (x1, y1 - 50), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 63, 216), 1)

                        # print(f"{depth_object_name}, {position_label}, {gimbal_label}")
                        output_text = f"{depth_object_name}, {position_label}, {gimbal_label}"


                # 터미널 출력 (덮어쓰기)
                sys.stdout.write(f"\r{output_text.ljust(100)}")  # 길이 고정하여 이전 값 삭제 방지
                sys.stdout.flush()

                cv2.imshow("Color Image", color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break


                if type(depth) == float and (not(math.isnan(depth)) and depth > 0.0):
                    if 0.3 <= depth <= 1.1:
                        self.temp_Velo = 200 * math.sqrt((depth - 0.3) / 0.8)
                    elif 0.0 < depth < 0.3:
                        # temp_Velo = -200 * math.sqrt((0.3 - depth) / 0.5) # 후진 적용 시
                        self.temp_Velo = 0
                    elif depth > 1.1:
                        self.temp_Velo = 0
                    else:
                        self.temp_Velo = 200
                    # print(type(depth))
                else: # Zero or Not a Number
                    self.temp_Velo = 0
                    # print(math.isnan(depth))
                
                
                if type(self.gimbal_angle_x) == float or int:
                    self.temp_Angle = FORWARD_POS + (self.max_deg_value * (self.gimbal_angle_x / 34.5))
                else:
                    pass

                # print(f"{depth:0.3f}, {gimbal_angle_x:6.2f}, {temp_Angle:2}")
                
                Pos_and_Velo = Dynmxl_process_for_ATOM(Outer_Wheel_Angle_in4096=self.temp_Angle, Outer_Wheel_Velocity_in200=self.temp_Velo)
                
                msg.data = f"Velocity: {Pos_and_Velo}, depth: {depth}"

                self.publisher.publish(msg)
                self.get_logger().info(f"발행됨: {msg.data}")


        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            Dynmxl_process_for_ATOM(Outer_Wheel_Angle_in4096=FORWARD_POS, Outer_Wheel_Velocity_in200=0)

    
def main():
    rclpy.init()
    atompub = atom_pub()

    try:
        while True:
            rclpy.spin(atompub) # 노드 실행 (이벤트 루프)
    except KeyboardInterrupt:
        pass
    finally:
        atompub.destroy_node() # 노드 정리
        rclpy.shutdown() # ROS 종료

if __name__ == '__main__':
    main()
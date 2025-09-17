# import time
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from std_msgs.msg import String
# from message_filters import ApproximateTimeSynchronizer, Subscriber
# from ultralytics import YOLO

# class UltralyticsNode(Node):
#     def __init__(self):
#         super().__init__('ultralytics')
#         time.sleep(1)

#         # YOLO 모델 로드
#         self.segmentation_model = YOLO("yolo11m-seg.pt")

#         # 퍼블리셔 생성
#         self.classes_pub = self.create_publisher(String, "/ultralytics/detection/distance", 10)

#         # 카메라 내부 파라미터 초기화
#         self.fx = None
#         self.fy = None
#         self.cx = None
#         self.cy = None

#         # 메시지 필터를 사용한 동기화
#         self.rgb_sub   = Subscriber(self, Image, "/camera/camera/color/image_raw")
#         self.depth_sub = Subscriber(self, Image, "/camera/camera/depth/image_raw")
#         self.info_sub  = Subscriber(self, CameraInfo, "/camera/camera/depth/camera_info")

#         self.ts = ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.1)
#         self.ts.registerCallback(self.combined_callback)

#     def image_to_numpy(self, img_msg):
#         """sensor_msgs.msg.Image를 NumPy 배열로 변환"""
#         if img_msg.encoding == 'rgb8':
#             dtype = np.uint8
#             n_channels = 3
#         elif img_msg.encoding == '16UC1':  # 깊이 이미지
#             dtype = np.uint16
#             n_channels = 1
#         else:
#             raise ValueError(f"Unsupported encoding: {img_msg.encoding}")

#         data = np.frombuffer(img_msg.data, dtype=dtype)
#         if n_channels > 1:
#             data = data.reshape((img_msg.height, img_msg.width, n_channels))
#         else:
#             data = data.reshape((img_msg.height, img_msg.width))
#         return data

#     def depth_to_xyz(self, depth, u, v, fx, fy, cx, cy):
#         """깊이 값과 픽셀 좌표를 이용해 XYZ 좌표 계산"""
#         Z = depth / 1000.0 if depth.dtype == np.uint16 else depth
#         X = (u - cx) * Z / fx
#         Y = (v - cy) * Z / fy
#         return X, Y, Z

#     def combined_callback(self, rgb_msg, depth_msg, info_msg):
#         """RGB, 깊이, 카메라 정보를 동기화하여 처리"""
#         if self.fx is None:
#             self.fx = info_msg.k[0]
#             self.fy = info_msg.k[4]
#             self.cx = info_msg.k[2]
#             self.cy = info_msg.k[5]
#             self.get_logger().info("Camera info received and set.")

#         # 이미지 변환
#         image = self.image_to_numpy(rgb_msg)
#         depth = self.image_to_numpy(depth_msg)

#         # YOLO 모델로 RGB 이미지 처리
#         result = self.segmentation_model(image)

#         all_objects = []
#         height, width = depth.shape
#         for index, cls in enumerate(result[0].boxes.cls):
#             class_index = int(cls.cpu().numpy())
#             name = result[0].names[class_index]
#             mask = result[0].masks.data.cpu().numpy()[index, :, :].astype(int)

#             # 마스크가 적용된 깊이 값 추출
#             obj_depth = depth[mask == 1]
#             obj_depth = obj_depth[~np.isnan(obj_depth)]
#             if len(obj_depth) == 0:
#                 continue

#             # 마스크가 적용된 픽셀 좌표 추출
#             v, u = np.where(mask == 1)
#             xyz_points = []
#             for i in range(len(u)):
#                 X, Y, Z = self.depth_to_xyz(obj_depth[i], u[i], v[i], self.fx, self.fy, self.cx, self.cy)
#                 xyz_points.append([X, Y, Z])

#             # XYZ 좌표의 평균 계산
#             xyz_points = np.array(xyz_points)
#             avg_xyz = np.mean(xyz_points, axis=0) if len(xyz_points) > 0 else [np.inf, np.inf, np.inf]

#             all_objects.append(f"{name}: X={avg_xyz[0]:.2f}m, Y={avg_xyz[1]:.2f}m, Z={avg_xyz[2]:.2f}m")

#         # 결과 발행
#         msg = String()
#         msg.data = str(all_objects)
#         self.classes_pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = UltralyticsNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down node")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main() 

# import rclpy 
# from rclpy.node import Node
# from std_msgs.msg import String
# from sensor_msgs.msg import Image, CameraInfo

# import numpy as np
# import ros2_numpy as rnp
# from ultralytics import YOLO

# from message_filters import Subscriber, ApproximateTimeSynchronizer

# # OpenCV for undistort (optional but recommended)
# import cv2

# class UltralyticsDepthXYZNode(Node):
#     def __init__(self):
#         super().__init__('ultralytics_depth_xyz')
#         self.get_logger().info("Start ultralytics_depth_xyz node")

#         # Model load
#         self.model = YOLO("yolo11m-seg.pt")

#         # Result publisher (JSON string for extensibility)
#         self.pub = self.create_publisher(String, '/ultralytics/detection/xyz', 10)

#         # Camera intrinsics initial values
#         self.fx = None
#         self.fy = None
#         self.cx = None
#         self.cy = None
#         self.dist_coeffs = None
#         self.camera_matrix = None

#         # camera_info subscription (async update)
#         self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.caminfo_cb, 10)

#         # message_filters: color + depth synchronization
#         self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
#         self.depth_sub = Subscriber(self, Image, '/camera/camera/depth/image_raw')
#         self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.05)
#         self.ts.registerCallback(self.synced_cb)

#     def caminfo_cb(self, msg: CameraInfo):
#         # Update camera matrix and distortion coeffs
#         K = np.array(msg.k).reshape(3,3)
#         self.fx = K[0,0]
#         self.fy = K[1,1]
#         self.cx = K[0,2]
#         self.cy = K[1,2]
#         self.camera_matrix = K
#         self.dist_coeffs = np.array(msg.d) if msg.d else None
#         self.get_logger().info(f"Got camera_info: fx={self.fx:.1f}, fy={self.fy:.1f}")

#     def synced_cb(self, color_msg: Image, depth_msg: Image):
#         # Check intrinsics readiness
#         if self.fx is None:
#             self.get_logger().warn("No camera_info yet — can't compute XYZ")
#             return

#         # Debug: Check received messages
#         self.get_logger().info(f"Color shape: {rnp.numpify(color_msg).shape if color_msg else 'None'}")
#         self.get_logger().info(f"Depth shape: {rnp.numpify(depth_msg).shape if depth_msg else 'None'}")

#         # NumPy conversion
#         color = rnp.numpify(color_msg)  # HxWx3
#         depth = rnp.numpify(depth_msg)  # HxW

#         # Depth unit: uint16 -> mm, float32 -> m
#         if depth.dtype == np.uint16:
#             depth_m = depth.astype(np.float32) * 0.001
#         else:
#             depth_m = depth.astype(np.float32)

#         # YOLO inference
#         res = self.model(color)
#         r0 = res[0]

#         # Masks if available
#         masks = r0.masks.data.cpu().numpy() if r0.masks is not None else None  # (N, H, W)

#         results_list = []

#         # For each detection
#         for idx, cls in enumerate(r0.boxes.cls):
#             cls_idx = int(cls.cpu().numpy())
#             name = r0.names[cls_idx]

#             if masks is None:
#                 results_list.append({"name": name, "x": None, "y": None, "z": None})
#                 continue

#             mask = masks[idx].astype(bool)  # HxW

#             # Valid mask: finite and >0
#             zs = depth_m[mask]
#             valid_mask = np.isfinite(zs) & (zs > 0)
#             zs = zs[valid_mask]

#             if zs.size == 0:
#                 results_list.append({"name": name, "x": None, "y": None, "z": None})
#                 continue

#             # Pixel coords from mask, filtered by valid_mask
#             ys_idx, xs_idx = np.nonzero(mask)
#             xs_idx = xs_idx[valid_mask]
#             ys_idx = ys_idx[valid_mask]

#             # (Optional) undistort if dist_coeffs available
#             if self.dist_coeffs is not None and np.any(self.dist_coeffs != 0):
#                 pts = np.stack([xs_idx.astype(np.float32), ys_idx.astype(np.float32)], axis=-1).reshape(-1,1,2)
#                 und = cv2.undistortPoints(pts, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
#                 xs_u = und[:,0,0]
#                 ys_u = und[:,0,1]
#             else:
#                 xs_u = xs_idx.astype(np.float32)
#                 ys_u = ys_idx.astype(np.float32)

#             # Vectorized XYZ calculation
#             Xs = (xs_u - self.cx) * zs / self.fx
#             Ys = (ys_u - self.cy) * zs / self.fy
#             Zs = zs

#             # Median for representative value (robust to outliers)
#             xm = float(np.median(Xs))
#             ym = float(np.median(Ys))
#             zm = float(np.median(Zs))

#             # Log XYZ coordinates
#             # self.get_logger().info(f"Object {name} (idx {idx}) x={xm:.2f}m, y={ym:.2f}m, z={zm:.2f}m")

#             results_list.append({"name": name, "x": xm, "y": ym, "z": zm})

#         # Log the results_list
#         self.get_logger().info(f"Results list: {results_list}")

#         # Publish as JSON
#         import json
#         msg = String()
#         msg.data = json.dumps(results_list)
#         self.pub.publish(msg)
#         self.get_logger().info(f"Published {len(results_list)} objects")

# def main(args=None):
#     rclpy.init(args=args)
#     node = UltralyticsDepthXYZNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class TopicPublisher(Node):
    def __init__(self):
        super().__init__('topic_publisher')
        self.ros_topics = {
            # '0': 'reset',
            '1': '/drive/button_start',
            '2': '/drive/fire_start_1',
            '3': '/drive/door_start',
            # '4': '/robot_arm/button_end',
            # '5': '/robot_arm/open_door_end',

            '4': '/drive/safebox_detection_again',
            '5': '/drive/human_start_1',
            '6': '/drive/human_start_2',
            '7': '/drive/human_start_3',
            '8': '/drive/finish_start', 
            '9': '/drive/fire_start_2',
            '0': '/drive/fire_start_3',
        }
        self.topic_publishers = {
            key: self.create_publisher(Empty, topic, 10)
            for key, topic in self.ros_topics.items()
        }
        self.get_logger().info('토픽 퍼블리셔 노드 시작. 1, 2, 3, 4, 5를 눌러 메시지 발행, q로 종료.')

    def publish_message(self, key):
        if key in self.ros_topics:
            topic = self.ros_topics[key]
            msg = Empty()
            self.topic_publishers[key].publish(msg)
            self.get_logger().info(f'{topic}에 메시지 발행')
        else:
            self.get_logger().warn(f'키 {key}는 유효한 토픽에 매핑되지 않음.')

def main():
    rclpy.init()
    node = TopicPublisher()
    try:
        while rclpy.ok():
            try:
                key = input("키를 입력하세요 (1, 2, 3, 4, 5 또는 q): ").strip()
                if key == 'q':
                    node.get_logger().info('프로그램 종료 요청됨.')
                    break
                elif key in node.ros_topics:
                    node.publish_message(key)
                else:
                    node.get_logger().warn(f'잘못된 입력: {key}. 유효한 키는 1, 2, 3, 4, 5 또는 q입니다.')
                rclpy.spin_once(node, timeout_sec=0.1)  # ROS2 콜백 처리
            except EOFError:
                node.get_logger().error('EOF 입력 감지. 프로그램 종료.')
                break
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트 감지. 프로그램 종료.')
    finally:
        node.get_logger().info('노드 종료 및 리소스 정리.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

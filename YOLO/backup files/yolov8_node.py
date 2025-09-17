# yolov8_node가 직접 파이프라인에서 카메라를 불러와서 이미지 처리.
# 인식되지 않으면 토픽 999를 발행.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2

class Yolov8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        self.image_publisher = self.create_publisher(
            Image, 
            '/object_detected', 
            10)
        self.class_id_publisher = self.create_publisher(
            Int32, 
            '/object_detected_class_id', 
            10)
        
        self.bridge = CvBridge()
        self.model = YOLO('/home/kminseo/Documents/mission/safebox/runs/detect/train6/weights/last.pt')
        self.get_logger().info("YOLOv8 Node Started")

        # RealSense 파이프라인 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.get_logger().info("RealSense pipeline started")

        # 0.1초(10Hz)마다 image_processing_callback 실행
        self.timer = self.create_timer(0.1, self.image_processing_callback)

    def image_processing_callback(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().warn("No color frame received")
            return

        frame = np.asanyarray(color_frame.get_data())
        results = self.model(frame)[0]

        detected_class_ids = set()

        # 박스 및 레이블 그리기
        if results.boxes is not None:
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = box.conf[0].cpu().numpy()
                cls_id = int(box.cls[0].cpu().numpy())

                if conf < 0.5:
                    continue

                detected_class_ids.add(cls_id)

                # 영상에 그림 그리는 부분 
                label = f"ID:{cls_id} Conf:{conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # 이미지 퍼블리시
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(img_msg)

        # 감지된 클래스 ID들 발행 (인식 하나만 발행)
        class_id_msg = Int32()
        if detected_class_ids:
            class_id_msg.data = list(detected_class_ids)[0]  # 첫 감지 ID 예시
        else:
            class_id_msg.data = 999
        self.class_id_publisher.publish(class_id_msg)




    def cleanup(self):
        self.pipeline.stop()
        self.get_logger().info("RealSense pipeline stopped")

def main(args=None):
    rclpy.init(args=args)
    node = Yolov8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
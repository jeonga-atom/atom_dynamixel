import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String # 이미지 형태로 보내야지 rviz에서 띄울 수 있음. 
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json

class Yolov8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        # self.subscription = self.create_subscription(   # 토픽 구독하는 메서드
        #     Image,
        #     '/camera/camera/color/image_raw',   # realsense 카메라에서 토픽 발행.
        #     self.image_callback,                # 수신된 메시지를 처리할 콜백 함수.
        #     10)
        self.publisher = self.create_publisher( # ROS2 노드에서 토픽을 발행하는 메서드.
            String, # 이미지를 발행해야함.-> image
            'yolov8/detections',                # 발행할 토픽 이름. 뭐가 잘못된 것인지 확인.
            10) 
        self.bridge = CvBridge()
        self.model = YOLO('yolov8s.pt')  # 원하는 모델 경로로 변경 가능
        self.get_logger().info("YOLOv8 Node Started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]

        detections = []

        if results and results.boxes is not None:
            for box, class_id, score in zip(results.boxes.xyxy.cpu(), results.boxes.cls.cpu(), results.boxes.conf.cpu()):
                if int(class_id) == 67:  # 관심 클래스 ID
                    x1, y1, x2, y2 = map(int, box.numpy())
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    detections.append({
                        'bbox': [x1, y1, x2, y2],
                        'score': float(score),
                        'class': int(class_id)
                    })

        if detections:
            msg = String()
            msg.data = json.dumps(detections)
            self.publisher.publish(msg)

        cv2.imshow('YOLOv8 Detection', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



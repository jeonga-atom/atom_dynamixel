import zmq
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

# 토픽 설정 (불변 상수)
DEFAULT_ROS_DETECTED = {
    'button': '/object_detection/button',
    'fire_on': '/object_detection/fire/on',
    'fire_off': '/object_detection/fire/off',
    'door': '/object_detection/door',
}

DEFAULT_ROS_DRIVE = {
    'button': '/drive/button_start',
    'fire': '/drive/fire_start',
    'door': '/drive/door_start',
}

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_publisher_node')

        # ROS 퍼블리셔 생성 (Empty 메시지)
        # 기본 QoS 사용 (필요하면 QoSProfile로 변경 가능)
        self.button_publisher = self.create_publisher(
            Empty,
            DEFAULT_ROS_DETECTED['button'],
            10)
        
        self.fire_on_publisher = self.create_publisher(
            Empty,
            DEFAULT_ROS_DETECTED['fire_on'],
            10)
        
        self.fire_off_publisher = self.create_publisher(
            Empty,
            DEFAULT_ROS_DETECTED['fire_off'],
            10)
        
        self.door_publisher = self.create_publisher(
            Empty,
            DEFAULT_ROS_DETECTED['door'],
            10)

        # ROS 서브스크라이버 생성
        self.button_subscriber = self.create_subscription(
            Empty,
            DEFAULT_ROS_DRIVE['button'],
            self.button_start_callback,
            10)
        
        self.fire_subscriber = self.create_subscription(
            Empty,
            DEFAULT_ROS_DRIVE['fire'],
            self.fire_start_callback,
            10)
        
        self.door_subscriber = self.create_subscription(
            Empty,
            DEFAULT_ROS_DRIVE['door'],
            self.door_start_callback,
            10)

        # ZeroMQ 컨텍스트 생성
        try:
            self.zmq_context = zmq.Context()

            # 탐지 소켓 (SUB)
            self.socket_detection = self.zmq_context.socket(zmq.SUB)
            self.socket_detection.connect("tcp://localhost:5556")
            self.socket_detection.setsockopt_string(zmq.SUBSCRIBE, "/object_detection/")

            # Drive 소켓 (PUB)
            self.socket_drive = self.zmq_context.socket(zmq.PUB)
            self.socket_drive.bind("tcp://*:5557")

            self.get_logger().info('ZeroMQ 컨텍스트 및 소켓 초기화 완료')

        except Exception as e:
            self.get_logger().error(f'ZeroMQ 초기화 실패: {str(e)}')
            raise

    # ---------- helper: 구독자 생길 때까지 기다렸다가 publish ----------
    def publish_wait_for_subscription(self, publisher, msg, key):
        """
        publisher.get_subscription_count()가 1 이상이 될 때까지 대기한 후 publish.
        rclpy.ok()가 False가 되면 루프 탈출합니다.
        """
        # 구독자가 생길 때까지 기다림 (무한 대기; 필요하면 timeout 로직 추가 가능)
        while publisher.get_subscription_count() == 0 and rclpy.ok():
            # spin_once로 콜백 처리 및 DDS 매칭 정보 갱신
            print(f"[Waiting] subscriber not found for {DEFAULT_ROS_DETECTED[key]} ...")
            rclpy.spin_once(self, timeout_sec=0.05) 

        # 구독자가 생겼거나 rclpy가 종료 신호를 받으면 발행
        publisher.publish(msg)
        # 발행 직후 DDS로 전송될 시간을 잠깐 보장
        rclpy.spin_once(self, timeout_sec=0.05)

    # ---------- Drive 콜백들 (ROS -> ZMQ) ----------
    def button_start_callback(self, msg):
        try:
            self.socket_drive.send_string("button_start")
            self.get_logger().info('ZMQ 발행: button_start')
        except Exception as e:
            self.get_logger().error(f'ZMQ button_start 발행 실패: {str(e)}')

    def fire_start_callback(self, msg):
        try:
            self.socket_drive.send_string("fire_start")
            self.get_logger().info('ZMQ 발행: fire_start')
        except Exception as e:
            self.get_logger().error(f'ZMQ fire_start 발행 실패: {str(e)}')

    def door_start_callback(self, msg):
        try:
            self.socket_drive.send_string("door_start")
            self.get_logger().info('ZMQ 발행: door_start')
        except Exception as e:
            self.get_logger().error(f'ZMQ door_start 발행 실패: {str(e)}')

    # ---------- ZMQ 수신 -> ROS 발행 루프 ----------
    def listen_and_publish(self):
        poller = zmq.Poller()
        poller.register(self.socket_detection, zmq.POLLIN)

        while rclpy.ok():
            try:
                socks = dict(poller.poll(timeout=10))  # 10ms 타임아웃 (ms)
                while self.socket_detection in socks and socks[self.socket_detection] == zmq.POLLIN:
                    message = self.socket_detection.recv_string(flags=zmq.NOBLOCK)
                    msg = Empty()
                    if message == "/object_detection/button":
                        # 구독자 생길 때까지 기다렸다가 발행하도록 변경
                        self.publish_wait_for_subscription(self.button_publisher, msg)
                        self.get_logger().info('ROS 발행: button_detected')

                    elif message == "/object_detection/fire/on":
                        self.publish_wait_for_subscription(self.fire_on_publisher, msg)
                        self.get_logger().info('ROS 발행: fire_on_detected')

                    elif message == "/object_detection/fire/off":
                        self.publish_wait_for_subscription(self.fire_off_publisher, msg)
                        self.get_logger().info('ROS 발행: fire_off_detected')

                    elif message == "/object_detection/door":
                        self.publish_wait_for_subscription(self.door_publisher, msg)
                        self.get_logger().info('ROS 발행: door_detected')

                    # 임시 토픽(/button_temp, /fire/on_temp 등)은 무시
                    socks = dict(poller.poll(timeout=0))

            except zmq.Again:
                pass
            
            except Exception as e:
                self.get_logger().error(f'메시지 수신 또는 발행 실패: {str(e)}')

            # 루프의 다른 콜백 처리를 위해 spin_once 호출 (짧게)
            rclpy.spin_once(self, timeout_sec=0.05)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        node.listen_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import zmq
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class Detected_Publisher(Node):
    def __init__(self):
        super().__init__('Detected_publisher')
        self.sos_publisher = self.create_publisher(
            Empty, 
            '/sos_detected', 
            10)
        
        # ZeroMQ 컨텍스트 생성
        try:
            self.zmq_context = zmq.Context()

            # sos 소켓
            self.zmq_socket_sos = self.zmq_context.socket(zmq.SUB)
            self.zmq_socket_sos.connect("tcp://localhost:5556")
            self.zmq_socket_sos.setsockopt_string(zmq.SUBSCRIBE, "/object_detection/")

            self.get_logger().info('ZeroMQ 컨텍스트 및 소켓 초기화 완료')
        
        except Exception as e:
            self.get_logger().error(f'ZeroMQ 초기화 실패: {str(e)}')
            raise

    def listen_and_publish(self):
        poller = zmq.Poller()
        poller.register(self.zmq_socket_sos, zmq.POLLIN)

        while rclpy.ok():
            try:
                socks = dict(poller.poll(timeout=10))  # 1초 타임아웃
                while self.zmq_socket_sos in socks and socks[self.zmq_socket_sos] == zmq.POLLIN:
                    message = self.zmq_socket_sos.recv_string(flags=zmq.NOBLOCK)
                    msg = Empty()
                    if message == "/object_detection/sos":
                        self.sos_publisher.publish(msg)
                        self.get_logger().info(f'ROS 발행: sos_detected')

                        # 임시 토픽(/button_temp, /fire/on_temp 등)은 무시
                        socks = dict(poller.poll(timeout=0))

            except zmq.Again:
                pass

            except Exception as e:
                self.get_logger().error(f'메시지 수신 또는 발행 실패: {str(e)}')
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = Detected_Publisher()
    try:
        node.listen_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
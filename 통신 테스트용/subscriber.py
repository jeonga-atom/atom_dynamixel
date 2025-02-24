import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.subscription = self.create_subscription(
            String,
            'animal_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"받음: {msg.data}")

def main():
    rclpy.init()
    node = KeyboardSubscriber()
    rclpy.spin(node)  # 계속 메시지 수신
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
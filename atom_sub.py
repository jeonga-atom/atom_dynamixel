import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

class atom_sub(Node):
    def __init__(self):
        super().__init__('atomsub')
        self.subscription = self.create_subscription(String, 'acc_topic', self.listener_callback, qos_profile_sensor_data)

    def listener_callback(self, msg):
        self.get_logger().info(f"받음: {msg.data}")

def main():
    rclpy.init()
    atomsub = atom_sub()
    rclpy.spin(atomsub)  # 계속 메시지 수신
    atomsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
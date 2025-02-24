import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import aioconsole

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'animal_topic', 10)

    async def read_keys(self):
        while rclpy.ok():
            key = await aioconsole.ainput("키 입력 (a: 고양이, b: 강아지, c: 토끼, q: 종료): ")
            msg = String()

            if key == 'a':
                msg.data = "고양이"
            elif key == 'b':
                msg.data = "강아지"
            elif key == 'c':
                msg.data = "토끼"
            elif key == 'q':
                self.get_logger().info("프로그램 종료")
                rclpy.shutdown()
                break
            else:
                continue

            self.publisher_.publish(msg)
            self.get_logger().info(f"발행됨: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.read_keys())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
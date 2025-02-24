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
            key = await aioconsole.ainput("키 입력 (a: 고양이, b: 강아지, c: 토끼, q: 종료): ")  # 비동기 키 입력
            msg = String()

            if key == 'a':
                msg.data = "고양이"
            elif key == 'b':
                msg.data = "강아지"
            elif key == 'c':
                msg.data = "토끼"
            elif key == 'q':  # 'q'를 누르면 종료
                self.get_logger().info("프로그램 종료")
                rclpy.shutdown()
                break
            else:
                continue  # 잘못된 입력이면 무시

            self.publisher_.publish(msg)  # 메시지 발행
            self.get_logger().info(f"발행됨: {msg.data}")

async def main():
    rclpy.init()
    node = KeyboardPublisher()

    task = asyncio.create_task(node.read_keys())  # 비동기 키 입력 실행
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())  # 비동기 실행

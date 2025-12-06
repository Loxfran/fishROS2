import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novel_queue = Queue()  # 创建队列 存储小说章节
        # 创建发布者, 发布小说章节
        self.novel_publisher = self.create_publisher(String, 'novel', 10)
        self.timer_ = self.create_timer(5.0, self.timer_callback)  # 每5秒发布一次章节

    def download_novel(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        self.get_logger().info("Novel downloaded successfully.")    
        for line in response.text.splitlines():
            self.novel_queue.put(line)
    
    def timer_callback(self):
        if not self.novel_queue.empty():
            novel_line = self.novel_queue.get()
            msg = String()
            msg.data = novel_line
            self.novel_publisher.publish(msg)
            self.get_logger().info(f"Published novel line: {msg.data}")


def main():
    rclpy.init()
    node = NovelPubNode("novel_pub")
    node.download_novel("http://0.0.0.0:8000/novel1.txt")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


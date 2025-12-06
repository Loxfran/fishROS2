import espeakng
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
import threading
import time

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novel_queue_ = Queue()  # 创建队列 存储小说章节
        # 创建发布者, 发布小说章节
        self.novel_subscriber_ = self.create_subscription(String, 'novel', self.novel_callback, 10)
        self.speech_thread = threading.Thread(target=self.speak_thread)
        self.speech_thread.start()  # 启动语音线程

    def novel_callback(self, msg):
        self.novel_queue_.put(msg.data)

    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'en'
        while rclpy.ok():
            if not self.novel_queue_.empty():
                novel_line = self.novel_queue_.get()
                self.get_logger().info(f"Received novel line: {novel_line}")
                speaker.say(novel_line)
                speaker.wait()
            else:
                time.sleep(1)  # 避免忙等待

def main():
    rclpy.init()
    node = NovelSubNode("novel_pub")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()





import rclpy
from rclpy.node import Node
from status_interface.msg import SystemStatus
import psutil
import platform


class SysStatusPub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.status_publisher = self.create_publisher(SystemStatus, 'system_status', 10)
        self.timer = self.create_timer(1, self.publish_status)
    
    def publish_status(self):
        cpu_percent = psutil.cpu_percent()
        memory_info = psutil.virtual_memory()
        net_io_counters = psutil.net_io_counters()

        message = SystemStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.host = platform.node()
        message.cpu_percent = cpu_percent
        message.memory_percent = memory_info.percent
        message.memory_total = memory_info.total /1024 /1024 /1024
        message.memory_avaiable = memory_info.available /1024 /1024 /1024
        message.net_sent = net_io_counters.bytes_sent /1024 /1024 /1024
        message.net_recv = net_io_counters.bytes_recv /1024 /1024 /1024

        self.get_logger().info(f'发布：{str(message)}')
        self.status_publisher.publish(message)


def main():
    rclpy.init()
    node = SysStatusPub('sys_status_pub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

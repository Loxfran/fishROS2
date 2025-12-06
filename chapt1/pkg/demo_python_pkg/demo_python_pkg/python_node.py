import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('python_node')
    node.get_logger().info('Hello World!')
    node.get_logger().warn('Hello World!')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

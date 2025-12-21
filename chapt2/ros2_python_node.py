import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    node = Node("python_node")
    node.get_logger().info("Hello, ROS2 from Python!")
    node.get_logger().warn("This is a warning message.")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

   
if __name__ == "__main__":
    main()
        
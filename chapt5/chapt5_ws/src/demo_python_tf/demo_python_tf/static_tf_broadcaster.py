import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.pub_static_tf()

    def pub_static_tf(self):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the header frame_id and child_frame_id
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        # Set the translation (x, y, z)
        t.transform.translation.x = 0.5
        t.transform.translation.y = 0.3
        t.transform.translation.z = 0.6

        # Set the rotation (roll, pitch, yaw)
        roll = math.radians(180)
        pitch = 0.0
        yaw = 0.0
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Publish the static transform
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform from world to robot_base')
    
def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
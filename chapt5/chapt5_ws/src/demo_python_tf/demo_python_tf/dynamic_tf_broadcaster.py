import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf2_broadcaster')
        self.dynamic_broadcaster = TransformBroadcaster(self)
        # self.pub_dynamic_tf()
        self.timer = self.create_timer(0.1, self.pub_dynamic_tf)

    def pub_dynamic_tf(self):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the header frame_id and child_frame_id
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'bottle_link'

        # Set the translation (x, y, z)
        t.transform.translation.x = 0.2
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.5

        # Set the rotation (roll, pitch, yaw)
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Publish the dynamic transform
        self.dynamic_broadcaster.sendTransform(t)
        self.get_logger().info('Published dynamic transform from world to robot_base')
    
def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
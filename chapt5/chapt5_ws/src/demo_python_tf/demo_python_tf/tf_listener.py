import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion


class TFListener(Node):
    def __init__(self):
        super().__init__('tf2_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.get_transform)

    def get_transform(self):
        try:
            # Lookup the transform from 'world' to 'robot
            relult =  self.tf_buffer.lookup_transform('base_link',
                                                     'bottle_link',
                                                     rclpy.time.Time(seconds=0),
                                                     rclpy.time.Duration(seconds=1.0))
            transform = relult.transform
            translation = transform.translation
            rotation = transform.rotation
            rotation_euler = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])    
            self.get_logger().info(f'Translation: x={translation.x}, y={translation.y}, z={translation.z}')
            self.get_logger().info(f'Rotation (radians): roll={rotation_euler[0]}, pitch={rotation_euler[1]}, yaw={rotation_euler[2]}')
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

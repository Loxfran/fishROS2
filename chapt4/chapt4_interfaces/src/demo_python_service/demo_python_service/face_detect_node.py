import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import face_recognition
import time
from rcl_interfaces.msg import SetParametersResult

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.bridge = CvBridge()
        self.srv = self.create_service(FaceDetector, '/face_detect', self.face_detect_callback)
        self.demo_image_path = get_package_share_directory('demo_python_service') + '/resource/demo.jpeg'
        self.declare_parameter('upsample_times', 1)
        self.declare_parameter('model', 'hog')
        self.upsample_times = self.get_parameter('upsample_times').get_parameter_value().integer_value
        self.model = self.get_parameter('model').get_parameter_value().string_value
        self.add_on_set_parameters_callback(self.params_callback)
        # self.upsample_times = 1
        # self.model = 'hog'

    def params_callback(self, params):
        for param in params:
            self.get_logger().info(f'params_callback: {param.name}->{param.value}')
            if param.name == 'upsample_times':
                self.upsample_times = param.value
            elif param.name == 'model':
                self.model = param.value
        return SetParametersResult(successful=True)

    def face_detect_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.demo_image_path)
        start_time = time.time()
        self.get_logger().info('Starting face detection...')
        face_locations = face_recognition.face_locations(cv_image, self.upsample_times, self.model)
        end_time = time.time()
        self.get_logger().info(f'Face detection completed in {end_time - start_time:.4f} seconds.')
        response.number = len(face_locations)
        response.use_time = end_time - start_time
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response 
    

def main():
    rclpy.init()
    node = FaceDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

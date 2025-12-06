import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FaceDetectionClient(Node):
    def __init__(self):
        super().__init__('face_detection_client')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        self.test_image_path = get_package_share_directory('demo_python_service') + '/resource/test1.jpg'
        self.image = cv2.imread(self.test_image_path)

    def send_request(self):
        while self.client.wait_for_service(timeout_sec=1.0) == False:
            self.get_logger().info('Service not available, waiting again...')
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'Number of faces detected: {response.number}, Time taken: {response.use_time:.4f} seconds')
        self.show_faces(response)

    def show_faces(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.imshow('Detected Faces', self.image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = FaceDetectionClient()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()
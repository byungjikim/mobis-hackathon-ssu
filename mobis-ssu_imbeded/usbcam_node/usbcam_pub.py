import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

class usbCamNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, '/camera/image', 10)
        self.cap = cv2.VideoCapture(0)  # USB 카메라에 연결되어 있는 경우 인덱스 0을 사용
        self.cv_bridge = CvBridge()


    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read image from camera')
            return
        image_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = usbCamNode()
    while rclpy.ok():
        camera_node.publish_image()
        
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
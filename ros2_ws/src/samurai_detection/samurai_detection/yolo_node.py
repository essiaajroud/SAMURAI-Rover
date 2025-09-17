import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'server/models/best2.pt')
        model_path = self.get_parameter('model_path').value
        
        # Load YOLO model
        self.model = torch.load(model_path)
        self.model.eval()
        if torch.cuda.is_available():
            self.model.cuda()
            
        # Publishers/Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)
            
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/samurai/detections',
            10)
            
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            # Process image with YOLO
            results = self.model(cv_image)
            
            # Convert results to ROS message
            det_msg = Detection2DArray()
            # ... convert YOLO results to Detection2D messages
            self.detections_pub.publish(det_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
            
    def depth_callback(self, msg):
        # Process depth information
        pass

def main():
    rclpy.init()
    node = YOLODetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

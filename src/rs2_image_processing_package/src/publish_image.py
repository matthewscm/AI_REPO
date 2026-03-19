#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class StaticImagePublisher(Node):
    def __init__(self):
        super().__init__('static_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()
        
        # Load the image
        self.img_path = 'blue_bottle.jpeg'
        if not os.path.exists(self.img_path):
            self.get_logger().error(f"Image not found at {self.img_path}")
            raise FileNotFoundError(f"Missing {self.img_path}")
            
        self.cv_image = cv2.imread(self.img_path)
        
        # Publish the image every 1 second (1.0 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f"Publishing {self.img_path} to /camera/color/image_raw...")

    def timer_callback(self):
        msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StaticImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
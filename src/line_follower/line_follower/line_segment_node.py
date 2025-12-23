#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineSegmentNode(Node):
    def __init__(self):
        super().__init__('line_segment_node')
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image, '/x3/camera/image_raw', self.image_callback, 10
        )
        self.error_pub = self.create_publisher(Float32, 'line_error', 10)
        self.conf_pub  = self.create_publisher(Float32, 'line_confidence', 10)
        
        self.kernel = np.ones((5, 5), np.uint8)
        self.min_area = 150  
        self.err_filt = 0.0
        self.err_alpha = 0.15  
        
        self.conf_filt = 0.0
        self.conf_alpha = 0.15  
        

        self.error_history = []
        self.history_size = 7  

        self.error_ma_history = []
        self.ma_size = 3
        
        self.get_logger().info("LineSegmentNode: Çok yumuşak 3-katmanlı filtreleme aktif")
    
    def detect(self, frame, crop_ratio):
        h, w, _ = frame.shape
        roi = frame[int(h * crop_ratio):, :]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        

        lower1 = np.array([0, 40, 30])      
        upper1 = np.array([15, 255, 255])   
        lower2 = np.array([165, 40, 30])    
        upper2 = np.array([180, 255, 255])
        
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower1, upper1),
            cv2.inRange(hsv, lower2, upper2)
        )
        
        mask = cv2.erode(mask, self.kernel, 1)
        mask = cv2.dilate(mask, self.kernel, 2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        c = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(c))
        
        if area < self.min_area:
            return None
        
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None
        
        cx = float(M["m10"] / M["m00"])
        error = (w / 2.0 - cx) / (w / 2.0)
        
        return float(error), area
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        
        res = self.detect(frame, 0.85)
        if res is None:
            res = self.detect(frame, 0.50)
        
        if res is None:
            return
        
        error, area = res
        

        self.error_history.append(error)
        if len(self.error_history) > self.history_size:
            self.error_history.pop(0)
        median_error = float(np.median(self.error_history))
        

        self.error_ma_history.append(median_error)
        if len(self.error_ma_history) > self.ma_size:
            self.error_ma_history.pop(0)
        ma_error = float(np.mean(self.error_ma_history))
        

        self.err_filt = (1 - self.err_alpha) * self.err_filt + self.err_alpha * ma_error
        self.conf_filt = (1 - self.conf_alpha) * self.conf_filt + self.conf_alpha * area
        
        self.error_pub.publish(Float32(data=float(self.err_filt)))
        self.conf_pub.publish(Float32(data=float(self.conf_filt)))

def main(args=None):
    rclpy.init(args=args)
    node = LineSegmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

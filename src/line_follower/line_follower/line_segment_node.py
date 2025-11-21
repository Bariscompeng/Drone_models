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

        camera_topic = '/x3/camera/image_raw'

        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        self.error_pub = self.create_publisher(Float32, 'line_error', 10)

    def image_callback(self, msg: Image):
        # ROS Image -> OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        # Alt yarıyı al (line zeminde olduğu için)
        roi = frame[int(h * 0.5):, :]

        # B, G, R kanallarını ayır
        b, g, r = cv2.split(roi)

        # int16'ya çevirip R - max(G,B) farkına bak (kırmızımsı pikseller)
        r16 = r.astype(np.int16)
        g16 = g.astype(np.int16)
        b16 = b.astype(np.int16)
        max_gb = np.maximum(g16, b16)

        # Şartlar:
        # 1) R, G ve B'den belirgin büyük (fark > 5)
        # 2) R değeri tamamen sıfıra yakın olmasın (burada 10 sınırı kullanıyoruz)
        red_like = (r16 - max_gb > 5) & (r16 > 10)

        # Maske: kırmızımsı pikseller 255, diğerleri 0
        mask = np.zeros_like(r, dtype=np.uint8)
        mask[red_like] = 255

        ys, xs = np.where(mask == 255)
        if len(xs) == 0:
            # Çizgi görünmüyor — uyar, ama node'u düşürme
            self.get_logger().warn("Line not detected")
            return

        # Çizgi merkezinin x koordinatı (ROI içinde)
        line_center_x = float(np.mean(xs))

        # Image'ın tüm genişliği üzerinden merkez alın
        image_center_x = w / 2.0
        pixel_error = image_center_x - line_center_x

        # Normalize [-1, 1]
        norm_error = pixel_error / (w / 2.0)

        msg_err = Float32()
        msg_err.data = norm_error
        self.error_pub.publish(msg_err)


def main(args=None):
    rclpy.init(args=args)
    node = LineSegmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


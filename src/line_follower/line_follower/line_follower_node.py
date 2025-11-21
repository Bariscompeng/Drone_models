import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        # Hata (line_error) subscriber
        self.error_sub = self.create_subscription(
            Float32,
            'line_error',
            self.error_callback,
            10
        )

        # Drone kontrol topic'i
        self.cmd_pub = self.create_publisher(
            Twist,
            '/model/M100/cmd_vel',   # topiğin doğru olduğundan eminsin zaten
            10
        )

        # Kontrol kazancı
        self.Kp = 0.8

        # Sabit ileri hız
        self.forward_speed = 0.3

        # Son hata ve son hata zamanı
        self.current_error = 0.0
        self.last_error_time = self.get_clock().now()

        # Kaç saniye boyunca line_error gelmezse duralım?
        self.lost_line_timeout = 0.5  # 0.5 saniye

        # Belirli aralıklarla control_loop çalışsın
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Line follower node started with timeout logic.")

    def error_callback(self, msg: Float32):
        # Her yeni hata geldiğinde kaydet ve zamanı güncelle
        self.current_error = msg.data
        self.last_error_time = self.get_clock().now()

    def control_loop(self):
        # Burada periyodik olarak cmd_vel basıyoruz
        now = self.get_clock().now()
        dt = (now - self.last_error_time).nanoseconds / 1e9  # saniye cinsinden

        cmd = Twist()

        if dt < self.lost_line_timeout:
            # Çizgi yakın zamanda görüldü → normal takip
            cmd.linear.x = self.forward_speed
            cmd.angular.z = self.Kp * self.current_error
        else:
            # Uzun süredir line_error gelmiyor → dur
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # (İstersen burada ufak bir arama davranışı da ekleyebilirsin:
            # cmd.angular.z = 0.3 vs. diyerek olduğu yerde dönsün.)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


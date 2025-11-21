import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

TARGET_FRAME = "base_link"   # IMU'yu bu frame'e sabitle

class ImuFrameFix(Node):
    def __init__(self):
        super().__init__('imu_frame_fix')
        self.sub = self.create_subscription(Imu, '/m100/imu', self.cb, 50)
        self.pub = self.create_publisher(Imu, '/m100/imu_fixed', 10)

    def cb(self, msg: Imu):
        msg.header.frame_id = TARGET_FRAME
        self.pub.publish(msg)

def main():
    rclpy.init()
    n = ImuFrameFix()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


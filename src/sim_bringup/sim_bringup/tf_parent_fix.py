import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

PARENT = "world"

class TfParentFix(Node):
    def __init__(self):
        super().__init__('tf_parent_fix')

        # use_sim_time paramını ikinci kez declare etme
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.sub = self.create_subscription(TFMessage, '/tf_raw', self.cb, qos)
        self.pub = self.create_publisher(TFMessage, '/tf', qos)

    def cb(self, msg: TFMessage):
        # Önce ROS (sim) saati, yoksa sistem saatine düş
        ros_now = self.get_clock().now()
        if ros_now.nanoseconds == 0:
            stamp = Clock(clock_type=ClockType.SYSTEM_TIME).now().to_msg()
        else:
            stamp = ros_now.to_msg()

        out = TFMessage()
        for t in msg.transforms:
            nt = TransformStamped()
            nt.header = t.header
            nt.child_frame_id = t.child_frame_id
            nt.transform = t.transform

            if not nt.header.frame_id:
                nt.header.frame_id = PARENT

            if nt.header.stamp.sec == 0 and nt.header.stamp.nanosec == 0:
                nt.header.stamp = stamp

            out.transforms.append(nt)

        if out.transforms:
            self.pub.publish(out)

def main():
    rclpy.init()
    n = TfParentFix()
    rclpy.spin(n)
    rclpy.shutdown()

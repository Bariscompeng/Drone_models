#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomFromGzPose(Node):
    def __init__(self):
        super().__init__('odom_from_gz_pose')

        # --- use_sim_time: tekrar declare etme; varsa dokunma ---
        # Eğer launch/cli ile verilmediyse default True yap:
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # ---- params ----
        self.declare_parameter('pose_topic', '/world/cave/dynamic_pose/info')
        self.declare_parameter('model_name', 'M100')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)

        self.pose_topic  = self.get_parameter('pose_topic').value
        self.model_name  = self.get_parameter('model_name').value
        self.odom_frame  = self.get_parameter('odom_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.pub_odom    = bool(self.get_parameter('publish_odom').value)
        self.pub_tf      = bool(self.get_parameter('publish_tf').value)

        # ---- pub/sub ----
        self.sub = self.create_subscription(TFMessage, self.pose_topic, self.cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10) if self.pub_odom else None
        self.tf_br = TransformBroadcaster(self) if self.pub_tf else None

        # log spam engeli
        self._warned_clock_zero = False
        self._warned_model_missing = False

        self.get_logger().info(
            f"Listening: {self.pose_topic} | model_name='{self.model_name}' -> /odom + TF({self.odom_frame}->{self.child_frame})"
        )

    def cb(self, msg: TFMessage):
        # 1) model transformunu bul
        t_model = None
        for t in msg.transforms:
            if t.child_frame_id == self.model_name:
                t_model = t
                break

        if t_model is None:
            if not self._warned_model_missing:
                self._warned_model_missing = True
                if msg.transforms:
                    sample = ', '.join([tr.child_frame_id for tr in msg.transforms[:8]])
                    self.get_logger().warn(
                        f"Model '{self.model_name}' not found in TFMessage. First frames: {sample}"
                    )
                else:
                    self.get_logger().warn("TFMessage empty.")
            return
        else:
            self._warned_model_missing = False

        # 2) sim time kontrolü
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            if not self._warned_clock_zero:
                self._warned_clock_zero = True
                self.get_logger().warn(
                    "Clock is 0. Waiting for /clock (use_sim_time true but sim time not active yet)."
                )
            return
        self._warned_clock_zero = False
        now_msg = now.to_msg()

        # 3) /odom publish
        if self.odom_pub is not None:
            odom = Odometry()
            odom.header.stamp = now_msg
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.child_frame

            odom.pose.pose.position.x = float(t_model.transform.translation.x)
            odom.pose.pose.position.y = float(t_model.transform.translation.y)
            odom.pose.pose.position.z = float(t_model.transform.translation.z)
            odom.pose.pose.orientation = t_model.transform.rotation

            # Twist yoksa 0 kalır (istersen sonradan delta ile ekleriz)
            self.odom_pub.publish(odom)

        # 4) TF publish (odom -> base_link)
        if self.tf_br is not None:
            tfm = TransformStamped()
            tfm.header.stamp = now_msg
            tfm.header.frame_id = self.odom_frame
            tfm.child_frame_id = self.child_frame
            tfm.transform.translation = t_model.transform.translation
            tfm.transform.rotation = t_model.transform.rotation
            self.tf_br.sendTransform(tfm)


def main():
    rclpy.init()
    node = OdomFromGzPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


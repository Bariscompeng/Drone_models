from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    static_tf_world_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )

    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    static_tf_odom_baselink = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    static_tf_baselink_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.1", "0", "0.1", "0", "0", "0", "base_link", "lidar_link"],
    )

    static_tf_baselink_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
    )

    return LaunchDescription([
        static_tf_world_map,
        static_tf_map_odom,
        static_tf_odom_baselink,
        static_tf_baselink_lidar,
        static_tf_baselink_imu,
    ])


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            'sim',
            'worlds',
            'cave1.sdf'
        ]),
        description='Path to SDF world file'
    )

    ign = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        output='screen',
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            'DRI_PRIME': '1',
            'IGN_RENDERING_ENGINE': 'ogre2',
        }
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_parameter_bridge',
        output='screen',
        arguments=[
            # CLOCK
            '/world/cave/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # IMU
            '/world/cave/model/M100/link/base_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # Camera
            '/x3/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

            # LiDAR
            '/x3/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # cmd_vel
            '/model/M100/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Pose
            '/world/cave/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/cave/clock', '/clock'),
            ('/world/cave/model/M100/link/base_link/sensor/imu_sensor/imu', '/m100/imu'),
        ],
    )

    # --- IMU frame fix ---
    imu_frame_fix = Node(
        package='sim_bringup',
        executable='imu_frame_fix',
        name='imu_frame_fix',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # --- ODOM from Gazebo pose ---
    odom_from_gz_pose = Node(
        package='sim_bringup',
        executable='odom_from_gz_pose',
        name='odom_from_gz_pose',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pose_topic': '/world/cave/dynamic_pose/info',
            'model_name': 'M100',
            'odom_frame': 'odom',
            'child_frame': 'base_link',
            'publish_odom': True,
            'publish_tf': True,
        }]
    )

    # --- 🔥 STATIC TF: base_link -> lidar_link ---
    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=[
            '0.10', '0.0', '0.135',   # x y z
            '0', '0', '0',           # roll pitch yaw
            'base_link',
            'lidar_link'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765}]
    )

    return LaunchDescription([
        world_arg,
        ign,
        bridge,
        imu_frame_fix,
        odom_from_gz_pose,
        static_lidar_tf,   # 👈 BURASI
        foxglove,
    ])


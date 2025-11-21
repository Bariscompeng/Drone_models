from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # World dosyası argümanı
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

    # Gazebo'yu çalıştır
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

    # ------------------------ STATIC TF ------------------------

    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'lidar_link']
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'M100/base_link/imu_sensor']
    )

    # ------------------------ CUSTOM FIX NODES ------------------------

    imu_frame_fix = Node(
        package='sim_bringup',
        executable='imu_frame_fix',
        name='imu_frame_fix',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tf_parent_fix = Node(
        package='sim_bringup',
        executable='tf_parent_fix',
        name='tf_parent_fix',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ------------------------ GZ-ROS BRIDGE ------------------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',

            '/world/cave/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',

            '/world/cave/model/M100/link/base_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu@gz.msgs.IMU',

            # Kamera — BURASI ÖNEMLİ
            '/x3/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',

            # LIDAR
            '/x3/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/model/M100/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        remappings=[
            ('/world/cave/dynamic_pose/info', '/tf_raw'),
            ('/world/cave/model/M100/link/base_link/sensor/imu_sensor/imu', '/m100/imu'),
        ],
        parameters=[
            {'qos_overrides./tf_static.publisher.durability': 'transient_local'}
        ]
    )

    # ------------------------ FOXGLOVE ------------------------

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765}]
    )

    # ------------------------ RETURN ------------------------

    return LaunchDescription([
        world_arg,
        ign,
        bridge,
        foxglove,
        tf_parent_fix,
        static_tf_imu,
        static_tf_lidar,
        imu_frame_fix
    ])


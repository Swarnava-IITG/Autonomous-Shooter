import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'robot_setup'
    pkg_share = FindPackageShare(pkg_name)
    workspace_root = os.path.expanduser('~/autonomous_shooter')

    world_file = os.path.join(workspace_root, 'world', 'warehouse.sdf')
    rviz_config_path = os.path.join(workspace_root, 'rviz', 'rviz_config.rviz')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'differential_bot.urdf.xacro'])
    map_yaml_file = os.path.join(workspace_root, 'maps', 'warehouse_layout.yaml')
    nav2_params_file = os.path.join(workspace_root, 'maps', 'nav2_config.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '-2.5', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    start_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/turret/pan_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/turret/tilt_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/my_robot/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose'
        ],
        output='screen'
    )

    run_shooter = Node(
        package="robot_setup",
        executable="detect_and_shoot",
        output="screen"
    )

    launch_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d",rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': nav2_params_file
        }.items()
    )

    delayed_spawn_and_bridge = TimerAction(
        period=3.0,
        actions=[spawn_robot, start_bridge]
    )

    delayed_rviz2 = TimerAction(
        period=3.0,
        actions=[launch_rviz,]
    )

    delayed_nav2 = TimerAction(
        period=3.0,
        actions=[start_nav2,]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        start_gazebo,
        robot_state_publisher_node,
        delayed_spawn_and_bridge,
        run_shooter,
        delayed_rviz2,
        delayed_nav2
    ])


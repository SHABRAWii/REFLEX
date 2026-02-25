from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')   # (not used below yet; keep for later if you want)
    paused = LaunchConfiguration('paused')       # (not used below yet; keep for later if you want)
    slow_factor = LaunchConfiguration('slow_factor')
    restore_factor = LaunchConfiguration('restore_factor')
    restore_factor_delay = LaunchConfiguration('restore_factor_delay')

    # NEW: Gazebo transport topic for IMU (world-spawned model usually publishes on a namespaced topic)
    gz_imu_topic = LaunchConfiguration('gz_imu_topic')

    pkg_share = FindPackageShare('reflex_gazebo')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', world])

    # Robot description for TF / controller_manager (if your gz_ros2_control subscribes to /robot_description)
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'reflex_gazebo.urdf.xacro'])
    xacro_exec = FindExecutable(name='xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                xacro_exec,
                TextSubstitution(text=' '),
                xacro_file,
                TextSubstitution(text=' '),
                TextSubstitution(text='prefix:=')
            ]),
            value_type=str
        )
    }

    # Gazebo
    gz_cmd_run = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        condition=UnlessCondition(paused)
    )

    gz_cmd_paused = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen',
        condition=IfCondition(paused)
    )

    # Bridge:
    # - /clock is always fine
    # - IMU: bridge the actual Gazebo IMU topic -> ROS /imu
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        # remappings=[
        #     ('/world/reflex_world/model/reflex/link/base_link/sensor/imu/imu', '/imu'),
        # ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # robot_state_publisher (useful for TF; also often publishes /robot_description topic for others)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # Load controllers
    load_controllers = [
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner',
                'joint_state_broadcaster', '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '120.0',
                '--switch-timeout', '60.0'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner',
                'diff_drive_controller', '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '120.0',
                '--switch-timeout', '60.0'],
            output='screen'
        ),
    ]

    slow_physics = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/reflex_world/set_physics',
                    '--reqtype', 'gz.msgs.Physics',
                    '--reptype', 'gz.msgs.Boolean',
                    '--req',
                    PythonExpression(["'real_time_factor: ' + str(", slow_factor, ")"])
                ],
                output='screen'
            )
        ]
    )

    restore_physics = TimerAction(
        period=restore_factor_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/reflex_world/set_physics',
                    '--reqtype', 'gz.msgs.Physics',
                    '--reptype', 'gz.msgs.Boolean',
                    '--req',
                    PythonExpression(["'real_time_factor: ' + str(", restore_factor, ")"])
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='reflex_starter.sdf'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('paused', default_value='False'),
        DeclareLaunchArgument('slow_factor', default_value='0.02'),
        DeclareLaunchArgument('restore_factor', default_value='1.0'),
        DeclareLaunchArgument('restore_factor_delay', default_value='15.0'),

        # IMPORTANT: set this to the REAL GZ IMU topic for the model spawned in the world
        DeclareLaunchArgument('gz_imu_topic', default_value='/imu'),

        gz_cmd_run,
        gz_cmd_paused,
        bridge,
        rsp,
        *load_controllers,
        slow_physics,
        restore_physics,
    ])

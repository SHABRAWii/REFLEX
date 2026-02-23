from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')   # (not used below yet; keep for later if you want)
    paused = LaunchConfiguration('paused')       # (not used below yet; keep for later if you want)

    controller_load_delay = LaunchConfiguration('controller_load_delay')
    unpause_delay = LaunchConfiguration('unpause_delay')

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
    gz_cmd = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen'
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

    # Ensure controller_manager sim time
    set_cm_sim_time = TimerAction(
        period=1.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'use_sim_time', 'true'],
                output='screen'
            )
        ]
    )

    # Load controllers
    load_controllers = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'controller_manager', 'spawner',
                    'joint_state_broadcaster', '-c', '/controller_manager',
                    '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'run', 'controller_manager', 'spawner',
                    'diff_drive_controller', '-c', '/controller_manager',
                    '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen'
            ),
        ]
    )

    # Unpause
    unpause = TimerAction(
        period=unpause_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/reflex_world/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', 'pause: false'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='reflex_starter.sdf'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('paused', default_value='True'),

        DeclareLaunchArgument('controller_load_delay', default_value='3.0'),
        DeclareLaunchArgument('unpause_delay', default_value='6.0'),

        # IMPORTANT: set this to the REAL GZ IMU topic for the model spawned in the world
        DeclareLaunchArgument('gz_imu_topic', default_value='/imu'),

        gz_cmd,
        bridge,
        rsp,
        set_cm_sim_time,
        load_controllers,
        unpause,
    ])
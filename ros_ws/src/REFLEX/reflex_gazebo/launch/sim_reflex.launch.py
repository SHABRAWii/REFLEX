from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch args
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    spawn_name = LaunchConfiguration('spawn_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    controller_load_delay = LaunchConfiguration('controller_load_delay')

    # Robot description from xacro
    pkg_share = FindPackageShare('reflex_gazebo')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'reflex_gazebo.urdf.xacro'])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro', ' ', xacro_file, ' ', 'prefix:=']),
            value_type=str
        )
    }

    # Gazebo command
    gz_cmd = ['gz', 'sim']
    gz_args = ['-r', world]
    # Note: keep GUI enabled for now. If you want true headless, we can add conditionals.

    return LaunchDescription([
        # -------- Launch Arguments --------
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('headless', default_value='False'),

        # Spawn pose defaults (change these as you like)
        DeclareLaunchArgument('spawn_name', default_value='reflex'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.8'),      # <-- higher Z by default
        DeclareLaunchArgument('spawn_roll', default_value='0.0'),
        DeclareLaunchArgument('spawn_pitch', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),

        # Delay controller loading until controller_manager is up
        DeclareLaunchArgument('controller_load_delay', default_value='3.0'),

        # -------- Start Gazebo --------
        ExecuteProcess(
            cmd=gz_cmd + gz_args,
            output='screen'
        ),

        # -------- robot_state_publisher --------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}]
        ),

        # -------- Spawn in Gazebo --------
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', spawn_name,
                '-topic', 'robot_description',
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
                '-R', spawn_roll,
                '-P', spawn_pitch,
                '-Y', spawn_yaw,
            ]
        ),
        # -------- Spawn in Gazebo --------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # -------- Load controllers (after a delay) --------
        TimerAction(
            period=controller_load_delay,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                         'joint_state_broadcaster'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                         'wheel_velocity_controller'],
                    output='screen'
                ),
            ]
        ),
    ])
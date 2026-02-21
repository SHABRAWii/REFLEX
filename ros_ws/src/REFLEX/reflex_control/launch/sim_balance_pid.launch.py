from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Pass-through args to sim_reflex.launch.py
    spawn_z = LaunchConfiguration('spawn_z')
    unpause_delay = LaunchConfiguration('unpause_delay')

    # Optional pass-through (keep if you want to override later)
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    controller_load_delay = LaunchConfiguration('controller_load_delay')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('reflex_gazebo'), 'launch', 'sim_reflex.launch.py'])
        ),
        launch_arguments={
            'spawn_z': spawn_z,
            'unpause_delay': unpause_delay,
            'world': world,
            'headless': headless,
            'controller_load_delay': controller_load_delay,
        }.items()
    )

    controller = Node(
        package='reflex_control',
        executable='reflex_balance_pid',
        output='screen',
        parameters=[{
            'left_joint': 'Left_Wheel_Joint',
            'right_joint': 'Right_Wheel_Joint',
            'wheel_radius': 0.06,
            'wheel_separation': 0.30,
            'balance_axis': 'pitch',
            'theta_max': 0.25,
            'w_max': 40.0,
            'inner_rate_hz': 250.0,
            'outer_rate_hz': 50.0,

            'v_kp': 0.6,
            'v_ki': 0.3,
            'v_kd': 0.0,
            'v_i_max': 0.3,
            'v_out_max': 0.25,

            't_kp': 25.0,
            't_ki': 0.0,
            't_kd': 1.2,
            't_i_max': 5.0,
            't_out_max': 40.0,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('spawn_z', default_value='1.0'),
        DeclareLaunchArgument('unpause_delay', default_value='6.0'),

        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('controller_load_delay', default_value='3.0'),

        sim,
        controller,
    ])
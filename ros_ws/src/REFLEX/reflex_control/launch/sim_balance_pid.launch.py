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
        name='reflex_balance_pid',
        output='screen',
        parameters=[{
            'left_joint': 'Left_Wheel_Joint',
            'right_joint': 'Right_Wheel_Joint',
            'wheel_radius': 0.06,
            'wheel_separation': 0.30,
            'balance_axis': 'roll',
            'theta_max': 0.25,
            'w_max': 40.0,
            'inner_rate_hz': 250.0,
            'outer_rate_hz': 50.0,

            'v_kp': -5.0,
            'v_ki': -3.0,
            'v_kd': 0.0,
            'v_i_max': 0.3,
            'v_out_max': 0.25,

            't_kp': -90.0,
            't_ki': 0.0,
            't_kd': 0.0,
            't_i_max': 5.0,
            't_out_max': 40.0,
        }]
    )

    tuner = Node(
        package='reflex_control',
        executable='reflex_pid_tuner',
        name='reflex_pid_tuner',
        output='screen',
        parameters=[{
            # Gazebo world/model (your sim uses world "default" and model "reflex")
            'gz_world': 'default',
            'model_name': 'reflex',

            # Default pose for teleport service (used when you call /apply_pose)
            'x': 0.0,
            'y': 0.0,
            'z': 1.0,
            'roll_deg': 0.0,
            'pitch_deg': 0.0,
            'yaw_deg': 0.0,

            # Don’t auto-teleport unless you want it:
            'apply_pose_on_start': False,

            # PID target node name
            'pid_node': '/reflex_balance_pid',

            # Default gains stored in tuner (used when you call /apply_gains)
            'v_kp': -5.0,
            'v_ki': -3.0,
            'v_kd': 0.0,
            'v_i_max': 0.3,
            'v_out_max': 0.25,

            't_kp': -90.0,
            't_ki': 0.0,
            't_kd': 0.0,
            't_i_max': 5.0,
            't_out_max': 40.0,

            # Don’t auto-apply gains unless you want it:
            'apply_gains_on_start': False,
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
        tuner,
    ])
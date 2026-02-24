from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Pass-through args to sim_reflex_starter.launch.py
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    paused = LaunchConfiguration('paused')

    spawn_name = LaunchConfiguration('spawn_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    controller_load_delay = LaunchConfiguration('controller_load_delay')
    unpause_delay = LaunchConfiguration('unpause_delay')
    plotter = Node(
        package='reflex_control',
        executable='reflex_debug_plotter',
        name='reflex_debug_plotter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'debug_topic': '/reflex_balance/debug',
            'window_sec': 15.0,
            'plot_hz': 20.0
        }]
    )
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('reflex_gazebo'),
                'launch',
                'sim_reflex_starter.launch.py'
            ])
        ),
        launch_arguments={
            'world': world,
            'headless': headless,
            'paused': paused,

            'spawn_name': spawn_name,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_roll': spawn_roll,
            'spawn_pitch': spawn_pitch,
            'spawn_yaw': spawn_yaw,

            'controller_load_delay': controller_load_delay,
            'unpause_delay': unpause_delay,
        }.items()
    )

    controller = Node(
        package='reflex_control',
        executable='reflex_balance_pid',
        name='reflex_balance_pid',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'left_joint': 'Left_Wheel_Joint',
            'right_joint': 'Right_Wheel_Joint',
            'wheel_radius': 0.06,
            'wheel_separation': 0.30,
            'balance_axis': 'roll',
            'theta_max': 1.5,
            'w_max': 3000.0,
            'inner_rate_hz': 250.0,
            'outer_rate_hz': 50.0,

            'v_kp': 0.5,
            'v_ki': 0.125,
            'v_kd': 0.0,
            'v_i_max': 0.3,
            'v_out_max': 1.5,

            't_kp': 1.7,
            't_ki': 0.5,
            't_kd': 0.0,
            't_i_max': 5.0,
            't_out_max': 5.0,
        }]
    )

    tuner = Node(
        package='reflex_control',
        executable='reflex_pid_tuner',
        name='reflex_pid_tuner',
        output='screen',
        parameters=[{
            # IMPORTANT: world name is reflex_world in reflex_starter.sdf
            'gz_world': 'reflex_world',
            'model_name': 'reflex',

            # Default pose for teleport service (used when you call /apply_pose)
            'x': 0.0,
            'y': 0.0,
            'z': 0.5,
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
        # ---- Sim args (defaults match your new setup) ----
        DeclareLaunchArgument('world', default_value='reflex_starter.sdf'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('paused', default_value='True'),

        DeclareLaunchArgument('spawn_name', default_value='reflex'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.5'),
        DeclareLaunchArgument('spawn_roll', default_value='0.0'),
        DeclareLaunchArgument('spawn_pitch', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),

        DeclareLaunchArgument('controller_load_delay', default_value='3.0'),
        DeclareLaunchArgument('unpause_delay', default_value='6.0'),

        # ---- Start everything ----
        sim,
        controller,
        tuner,
        # plotter,
    ])
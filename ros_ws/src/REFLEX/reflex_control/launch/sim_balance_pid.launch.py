from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('reflex_gazebo'), 'launch', 'sim_reflex.launch.py'])
        )
    )

    controller = Node(
        package='reflex_control',
        executable='reflex_balance_pid',
        output='screen',
        parameters=[{
            'left_joint': 'Left_Wheel_Joint',
            'right_joint': 'Right_Wheel_Joint',
            'wheel_radius': 0.06,       # <-- set yours
            'wheel_separation': 0.30,   # <-- set yours
            'balance_axis': 'pitch',    # recommended
            'theta_max': 0.25,
            'w_max': 40.0,
            'inner_rate_hz': 250.0,
            'outer_rate_hz': 50.0,

            # Outer PID (velocity->theta_set)
            'v_kp': 0.6,
            'v_ki': 0.3,
            'v_kd': 0.0,
            'v_i_max': 0.3,
            'v_out_max': 0.25,

            # Inner PID (theta->wheel)
            't_kp': 25.0,
            't_ki': 0.0,
            't_kd': 1.2,
            't_i_max': 5.0,
            't_out_max': 40.0,
        }]
    )

    return LaunchDescription([sim, controller])
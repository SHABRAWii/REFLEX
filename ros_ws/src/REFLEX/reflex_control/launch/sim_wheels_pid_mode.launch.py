from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('reflex_gazebo'), 'launch', 'sim_reflex.launch.py'])
        )
    )

    pid_node = Node(
        package='reflex_control',
        executable='wheel_velocity_pid',
        output='screen',
        parameters=[{
            'left_joint': 'Left_Wheel_Joint',
            'right_joint': 'Right_Wheel_Joint',
            'kp': 0.8,
            'ki': 2.0,
            'kd': 0.0,
            'i_max': 5.0,
            'out_max': 20.0,
            'rate_hz': 250.0,
        }]
    )

    return LaunchDescription([sim_launch, pid_node])
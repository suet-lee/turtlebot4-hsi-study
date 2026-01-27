from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hsi_trial_control',
            executable='hsi_trial_control',
            name='hsi_trial_control',
            # namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,  # Ensures output is similar to direct terminal execution
            # parameters=[{
            # }],
            arguments=[],
        )
    ])

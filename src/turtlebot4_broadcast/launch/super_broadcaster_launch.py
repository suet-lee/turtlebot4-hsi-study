from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'env',
            default_value='qualisys',
            description='Source of positional data: real-world tracking system or simulation',
        ),
        Node(
            package='turtlebot4_broadcast',
            executable='super_broadcaster_node',
            name='super_broadcaster_node',
            # namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,  # Ensures output is similar to direct terminal execution
            parameters=[{
                'env': LaunchConfiguration('env'),
            }],
            arguments=[],
        )
    ])

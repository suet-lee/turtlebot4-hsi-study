from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Existing namespace argument declaration
        DeclareLaunchArgument(
            'namespace',
            default_value='',
        ),
        Node(
            package='turtlebot4_team',
            executable='shuffle_node',
            name='robot_shuffle',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,  # Ensures output is similar to direct terminal execution
            # Pass parameters to the node
            # parameters=[{
            # }],
            arguments=[],
        )
    ])

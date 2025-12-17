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
        DeclareLaunchArgument(
            'max_active_zones',
            default_value='4',
            description='Maximum number of active zones',
        ),
        Node(
            package='turtlebot4_team',
            executable='task_node',
            name='task_manager',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,  # Ensures output is similar to direct terminal execution
            # Pass parameters to the node, including the n_teams parameter
            parameters=[{
                'max_active_zones': LaunchConfiguration('max_active_zones'),
            }],
            arguments=[],
        )
    ])

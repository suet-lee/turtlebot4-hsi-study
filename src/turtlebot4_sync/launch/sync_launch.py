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
            'center_x',
            default_value='0',
            description='Center of mill, x coord',
        ),
        DeclareLaunchArgument(
            'center_y',
            default_value='0',
            description='Center of mill, y coord',
        ),
        Node(
            package='turtlebot4_sync',
            executable='sync_node',
            name='robot_sync',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,  # Ensures output is similar to direct terminal execution
            # Pass parameters to the node
            parameters=[{
                'center_x': LaunchConfiguration('center_x'),
                'center_y': LaunchConfiguration('center_y')
            }],
            arguments=[],
        )
    ])

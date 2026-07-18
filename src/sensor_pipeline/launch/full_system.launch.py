from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Declare arguments (command line inputs)
        DeclareLaunchArgument('degraded_threshold', default_value='3'),
        DeclareLaunchArgument('estop_threshold', default_value='5'),
        DeclareLaunchArgument('fault_burst_start', default_value='10'),
        DeclareLaunchArgument('fault_burst_end', default_value='15'),

        # 2. Launch Publisher Node
        Node(
            package='sensor_pipeline',
            executable='sensor_publisher_node',
            name='sensor_publisher',
            parameters=[{
                'fault_burst_start': LaunchConfiguration('fault_burst_start'),
                'fault_burst_end': LaunchConfiguration('fault_burst_end')
            }]
        ),

        # 3. Launch Monitor Node
        Node(
            package='sensor_pipeline',
            executable='safety_monitor_node',
            name='safety_monitor',
            parameters=[{
                'degraded_threshold': LaunchConfiguration('degraded_threshold'),
                'estop_threshold': LaunchConfiguration('estop_threshold')
            }]
        )
    ])
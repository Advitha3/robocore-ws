from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('degraded_threshold', default_value='3'),
        DeclareLaunchArgument('estop_threshold', default_value='5'),

        # Physics Simulator
        Node(
            package='drone_control',
            executable='drone_simulator_node',
            name='simulator'
        ),
        
        # Extended Kalman Filter
        Node(
            package='drone_control',
            executable='ekf_node',
            name='ekf'
        ),
        
        # PID Controller
        Node(
            package='drone_control',
            executable='pid_node',
            name='pid'
        ),

        # Safety Monitor
        Node(
            package='sensor_pipeline',
            executable='safety_monitor_node',
            name='safety',
            parameters=[{
                'degraded_threshold': LaunchConfiguration('degraded_threshold'),
                'estop_threshold': LaunchConfiguration('estop_threshold')
            }]
        )
    ])
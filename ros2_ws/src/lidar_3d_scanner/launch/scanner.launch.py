from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for STM32'
        ),
        
        Node(
            package='lidar_3d_scanner',
            executable='scanner_node',
            name='scanner_node',
            parameters=[
                {'port': LaunchConfiguration('port')}
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('lidar_3d_scanner'), 'config', 'scanner.rviz')]
        )
    ])

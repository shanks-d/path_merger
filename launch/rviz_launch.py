import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('path_merger')
    
    return LaunchDescription([
        Node(
            package='path_merger',
            executable='visualize',
            name='visualize_paths'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(pkg_dir, 'config', 'rviz_config.rviz')]
        )
    ])
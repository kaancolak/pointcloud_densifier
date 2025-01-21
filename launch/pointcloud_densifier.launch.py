from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_prefix = get_package_share_directory('pointcloud_densifier')
    
    config_file = os.path.join(pkg_prefix, 'config/pointcloud_densifier.param.yaml')

    return LaunchDescription([
        Node(
            package='pointcloud_densifier',
            executable='pointcloud_densifier_node',
            name='pointcloud_densifier',
            parameters=[config_file],
            remappings=[
                ('input', '/sensing/lidar/concatenated/pointcloud'),
                ('output', '/perception/pointcloud_densifier/pointcloud')
            ],
        )
    ])
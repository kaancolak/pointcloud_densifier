from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_prefix = get_package_share_directory('pointcloud_densifier')
    config_file = os.path.join(pkg_prefix, 'config/pointcloud_densifier.param.yaml')

    # Declare arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/sensing/lidar/concatenated/pointcloud',
        description='Input pointcloud topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/perception/pointcloud_densifier/pointcloud',
        description='Output pointcloud topic'
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        Node(
            package='pointcloud_densifier',
            executable='pointcloud_densifier_node',
            name='pointcloud_densifier',
            parameters=[config_file],
            remappings=[
                ('input', LaunchConfiguration('input_topic')),
                ('output', LaunchConfiguration('output_topic'))
            ],
        )
    ])
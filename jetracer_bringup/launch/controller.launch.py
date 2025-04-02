from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('jetracer_bringup'),
        'config',
        'controller_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='jetracer_controller',
            executable='jetracer_controller_node',
            name='jetracer_controller_node',
            output='screen',
            parameters=[config]
        )
    ])
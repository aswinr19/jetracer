from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_namespace = LaunchConfiguration('ns')
    navigation_launch_file_phat = os.path.join(
        get_package_share_directory('jetracer_bringup'),
        'launch',
        'navigation_launch.py'
    )

    navigation_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([navigation_launch_file_phat]),
        launch_arguments=[
            ['namespace', robot_namespace],
            ['use_sim_time', 'false']
        ]
    )
    namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='namespace of robot',
    )

    return LaunchDescription([
        namespace_arg,
        navigation_launch_desc
    ])
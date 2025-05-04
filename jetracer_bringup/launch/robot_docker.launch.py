from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_namespace = LaunchConfiguration('ns')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map = LaunchConfiguration('map')
    
    namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='namespace of robot',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        description='if to use sim time'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        description='map to use'
    )

    localization_launch_file_phat = os.path.join(
        get_package_share_directory('jetracer_bringup'),
        'launch',
        'localization_launch.py'
    )
    
    navigation_launch_file_phat = os.path.join(
        get_package_share_directory('jetracer_bringup'),
        'launch',
        'navigation_launch.py'
    )

    localization_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([localization_launch_file_phat]),
        launch_arguments=[
            ['namespace', robot_namespace],
            ['use_sim_time', use_sim_time],
            ['map', map]
        ]
    )

    navigation_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([navigation_launch_file_phat]),
        launch_arguments=[
            ['namespace', robot_namespace],
            ['use_sim_time', use_sim_time]
        ]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        map_arg,
        localization_launch_desc,
        navigation_launch_desc
    ])
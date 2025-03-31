import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    namespace_robot = LaunchConfiguration('ns')
    namespace_argrobot = DeclareLaunchArgument(
        'ns',
        description='Namespace of robot'
    )

    default_params_file = os.path.join(get_package_share_directory("jetracer_bringup"),
                                       'config', 'mapper_params_online_async.yaml')
    
    start_async_slam_toolbox_node = Node(
        parameters=[
          default_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace= namespace_robot,
        name='slam_toolbox',
        output='screen',
        remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/scan', 'scan'),
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata'),
                ('/map_updates', 'map_updates')
            ],
        )

    return LaunchDescription([
        namespace_argrobot,
        use_sim_time_argument,
        start_async_slam_toolbox_node
    ])
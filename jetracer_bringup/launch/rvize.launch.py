from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Declare the namespace argument
    namespace_argrobot = DeclareLaunchArgument(
        'ns',
        description='Namespace for tf topics'
    )
    namespace_robot = LaunchConfiguration('ns')

    rviz_config_file = os.path.join(
        get_package_share_directory('jetracer_bringup'), 'rviz', 'jetracer.rviz'
    )


    return LaunchDescription([
        namespace_argrobot,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace_robot,
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/camera/image_raw', 'camera/image_raw'),
                ('/camera/camera_info', 'camera/camera_info'),
                ('/camera/image_raw/compressed', 'camera/image_raw/compressed'),
                ('/camera/image_raw/compressedDepth', 'camera/image_raw/compressedDepth'),
                ('/camera/image_raw/theora', 'camera/image_raw/theora'),
                ('/slam_toolbox/clear_changes', 'slam_toolbox/clear_changes'),
                ('/slam_toolbox/describe_parameters', 'slam_toolbox/describe_parameters'),
                ('/slam_toolbox/deserialize_map', 'slam_toolbox/deserialize_map'),
                ('/slam_toolbox/dynamic_map', 'slam_toolbox/dynamic_map'),
                ('/slam_toolbox/get_interactive_markers', 'slam_toolbox/get_interactive_markers'),
                ('/slam_toolbox/get_parameter_types', 'slam_toolbox/get_parameter_types'),
                ('/slam_toolbox/get_parameters', 'slam_toolbox/get_parameters'),
                ('/slam_toolbox/list_parameters', 'slam_toolbox/list_parameters'),
                ('/slam_toolbox/manual_loop_closure', 'slam_toolbox/manual_loop_closure'),
                ('/slam_toolbox/pause_new_measurements', 'slam_toolbox/pause_new_measurements'),
                ('/slam_toolbox/save_map', 'slam_toolbox/save_map'),
                ('/slam_toolbox/serialize_map', 'slam_toolbox/serialize_map'),
                ('/slam_toolbox/set_parameters', 'slam_toolbox/set_parameters'),
                ('/slam_toolbox/set_parameters_atomically', 'slam_toolbox/set_parameters_atomically'),
                ('/slam_toolbox/toggle_interactive_mode', 'slam_toolbox/toggle_interactive_mode')
                ],
            output='screen',
            arguments=[
                "-d", rviz_config_file
            ]
        )
    ])

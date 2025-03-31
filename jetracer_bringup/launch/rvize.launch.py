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
                ('/camera/image_raw/theora', 'camera/image_raw/theora')
                ],
            output='screen',
            arguments=[
                "-d", rviz_config_file
            ]
        )
    ])

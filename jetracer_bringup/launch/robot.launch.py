from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_namespace = LaunchConfiguration('ns')
    config = os.path.join(
        get_package_share_directory('jetracer_bringup'),
        'config',
        'controller_config.yaml'
    )
    joystick_launch_file_phat = os.path.join(
        get_package_share_directory('jetracer_bringup'),
        'launch',
        'joystick.launch.py'
    )
    description_launch_file_phat = os.path.join(
        get_package_share_directory('jetracer_description'),
        'launch',
        'jetracer_description.launch.py'
    )
    rplidar_launch_file_phat = os.path.join(
        get_package_share_directory('jetracer_bringup'),
        'launch',
        'rplidar_a1_launch.py'
    )

    joystick_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([joystick_launch_file_phat]),
        launch_arguments=[
            ['controller', 'robot'],
            ['ns', robot_namespace]
        ]
    )

    description_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([description_launch_file_phat]),
        launch_arguments=[
            ['ns', robot_namespace],
            ['sim', 'false']
        ]
    )

    rplidar_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([rplidar_launch_file_phat]),
        launch_arguments=[
            ['ns', robot_namespace],
            ['serial_baudrate', '/dev/ttyACM1'],
            ['serial_baudrate', '115200'],
            ['frame_id', 'laser'],
            ['inverted', 'false'],
            ['angle_compensate', 'true']
        ]
    )

    namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='namespace of robot',
    )

    controller_node = Node(
            package='jetracer_controller',
            executable='jetracer_controller_node',
            name='jetracer_controller_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[config]
        )

    return LaunchDescription([
        namespace_arg,
        controller_node,
        joystick_launch_desc,
        description_launch_desc,
        rplidar_launch_desc
    ])
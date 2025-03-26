from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # Declare the namespace argument
    namespace_argrobot = DeclareLaunchArgument(
        'ns',
        description='Namespace for tf topics'
    )
    namespace_robot = LaunchConfiguration('ns')

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
                ],
            output='screen'
        )
    ])

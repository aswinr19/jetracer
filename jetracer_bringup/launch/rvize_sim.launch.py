from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument(
        'ns',
        description='Namespace for tf topics'
    )
    namespace = LaunchConfiguration('ns')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            remappings=[
                ('/tf', PathJoinSubstitution([namespace, 'tf'])),
                ('/tf_static', PathJoinSubstitution([namespace, 'tf_static'])),
                ],
            output='screen'
        )
    ])

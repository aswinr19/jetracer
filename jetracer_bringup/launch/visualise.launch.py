import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration , PythonExpression , Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    main_xacro_file = os.path.join(
        get_package_share_directory('jetracer_description'),'urdf','jetracer_main.xacro'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('jetracer_bringup'), 'rviz', 'visualise_config.rviz'
    )

    if os.path.exists(main_xacro_file):
        jetracer_description = Command(
            # spaces are important here since this is if we entered it on the CLI
            ["xacro"," ",main_xacro_file]
        )
    else:
        print(f'failed to open {main_xacro_file}')
        exit()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(jetracer_description,value_type=str)
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                "-d", rviz_config_file
            ]
        )
    ])
        


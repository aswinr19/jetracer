import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration , Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_namespace = LaunchConfiguration('ns')
    robot_namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='[ARG] required namespace to keep nodes and topics separate when running multiple robots in the same simulation'
    )

    sim = LaunchConfiguration('sim')
    sim_arg = DeclareLaunchArgument(
        name='sim',
        default_value='true',
        description='[ARG] tells the robot_state_publihser to use the simulation model'
    )

    main_xacro_file = os.path.join(
        get_package_share_directory('jetracer_description'),'urdf','jetracer_main.xacro'
    )

    if os.path.exists(main_xacro_file):
        jetracer_description = Command(
            # spaces are important here since this is if we entered it on the CLI
            ["xacro"," ",main_xacro_file," ","namespace:=",robot_namespace, " use_sim:=", sim]
        )
    else:
        print(f'failed to open {main_xacro_file}')
        exit()

    return LaunchDescription([
        robot_namespace_arg,
        sim_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=robot_namespace,
            parameters=[{
                'robot_description': ParameterValue(jetracer_description,value_type=str),
                'use_sim_time': sim
            }],
            remappings=[
                ('/tf','tf'),
                ('/tf_static','tf_static'),
            ]
        ),   
    ])
        


import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration , PythonExpression , Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='[ARG] tells the robot_state_publisher to use the simulation time or just unix timestamp'
    )

    robot_namespace = LaunchConfiguration('ns')
    robot_namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='[ARG] required namespace to keep nodes and topics separate when running multiple robots in the same simulation'
    )


    main_xacro_file = os.path.join(
        get_package_share_directory('jetracer_description'),'urdf','jetracer_main.xacro'
    )

    if os.path.exists(main_xacro_file):
        jetracer_description = Command(
            # spaces are important here since this is if we entered it on the CLI
            ["xacro"," ",main_xacro_file," ","namespace:=",robot_namespace]
        )
    else:
        print(f'failed to open {main_xacro_file}')
        exit()

    return LaunchDescription([
        use_sim_time_arg,
        robot_namespace_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=robot_namespace,
            parameters=[{
                'robot_description': ParameterValue(jetracer_description,value_type=str),
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('/tf','tf'),
                ('/tf_static','tf_static'),
            ]
        ),   
    ])
        


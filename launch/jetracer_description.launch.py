# required to get the Xacro files
import os
import xacro
# main libraries used in our ".launch" files to provide the base classes
# LaunchDescription , Node
from launch import LaunchDescription
from launch_ros.actions import Node
# in order to retrieve the "/share" directory of a given package
# e.g.: a package in "/home/user/ros_ws/install" using this function with
#       argument "jetracer_bringup" will return
#       "/home/user/ros_ws/install/share/jetracer_bringup/"
from ament_index_python.packages import get_package_share_directory

# explained in later packages...
# but for now, it just allows us to pass arguments to this Launch file/script
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # explained in later packages...
    # but for now,
    # - LaunchConfiguration : enables us to acquire a launch argument called 'use_sim_time'
    # - DeclareLaunchArgument : enables us to pass arguments from above launch file or console
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='[ARG] tells the robot_state_publisher to use the simulation time or just unix timestamp'
    )

    # there is always 1 xacro file that includes all other xacro files
    # so we only need to find that one and ROS's xacro tool will take care
    # of the rest for us. This main xacro file is located (in our situation at least)
    # in the URDF directory
    main_xacro_file = os.path.join(
        get_package_share_directory(
            'jetracer_bringup'), 'urdf', 'jetracer_main.xacro'
    )

    # of course we have to check if we actually have a file
    if os.path.exists(main_xacro_file):
        jetracer_description = xacro.process_file(main_xacro_file).toxml()
    else:
        print(f'failed to open {main_xacro_file}')
        exit()

    # finally return a LaunchDescription with the configured nodes
    # NOTE: here we CONFIGURE the node NOT CREATE them.
    # they were created in their respective packages
    return LaunchDescription([
        use_sim_time_arg,
        # first node node launches the "robot_state_publisher"
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                # parameters are very important for other programs such as RViz or ros_gz_bridge
                # to read the correct data they will use these exact names published by this Node
                'robot_description': jetracer_description,
                # this parameter will be usefull later on
                'use_sim_time': use_sim_time
            }]
        )
    ])

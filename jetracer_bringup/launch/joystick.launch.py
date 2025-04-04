from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_namespace = LaunchConfiguration('ns')
    controller = LaunchConfiguration("controller")

    joy_params = PathJoinSubstitution([
        FindPackageShare('jetracer_bringup'),
        'config',
        PythonExpression([
            "'",
            TextSubstitution(text='joystick_'),
            controller,
            TextSubstitution(text='.yaml'),
            "'"
        ])
    ])

    twist_mux_params = PathJoinSubstitution([
        FindPackageShare('jetracer_bringup'),
        'config/twist_mutx.yaml'
    ])

    namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='namespace of robot',
    )

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='robot',
        description='controller type (xbox, robot)'
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            namespace=robot_namespace,
            parameters=[joy_params],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            namespace=robot_namespace,
            parameters=[joy_params],
            remappings=[('cmd_vel','cmd_vel_joy')]
         )
    
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=robot_namespace,
        parameters=[twist_mux_params],
        remappings=[('cmd_vel_out','cmd_vel_unstamped')]
    )

    return LaunchDescription([
        namespace_arg,
        controller_arg,
        joy_node,
        teleop_node,
        twist_mux
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('ns')
    controller = LaunchConfiguration("controller")

    # Use PathJoinSubstitution with TextSubstitution for proper string composition
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

    namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='namespace of robot',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
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
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            namespace=robot_namespace,
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel','cmd_vel_unstamped')]
         )

    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            namespace=robot_namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel_in','cmd_vel_unstamped'),
                        ('cmd_vel_out', 'ackermann_steering_controller/reference')]
         )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        controller_arg,
        joy_node,
        teleop_node,
        twist_stamper,
    ])

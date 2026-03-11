from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Declare the simple argument
    angle_arg = DeclareLaunchArgument(
        'angle',
        default_value='90',
        description='Startup angle: 0, 90, or 180'
    )

    # 2. Nodes
    streamer_node = Node(
        package='robot_control',
        executable='stream',
        output='screen'
    )

    ml_node = Node(
        package='robot_control',
        executable='image_scan',
        output='screen'
    )

    # Servo Node now takes the parameter directly
    servo_node = Node(
        package='robot_control',
        executable='servo_move',
        name='cam_manager',
        parameters=[{'initial_angle': LaunchConfiguration('angle')}],
        output='screen'
    )

    # Delayed Line Follower
    delayed_line_follower = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='robot_control',
                executable='line_follower',
                output='log'
            )
        ]
    )

    return LaunchDescription([
        angle_arg,
        streamer_node,
        ml_node,
        servo_node,
        delayed_line_follower
    ])

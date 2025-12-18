from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    command = LaunchConfiguration('command')

    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='1',
        description='TurtleBot robot index (1 → /robot1)'
    )

    declare_command = DeclareLaunchArgument(
        'command',
        default_value='undock',
        description='dock or undock'
    )

    docking_node = Node(
        package='twocops_bringup',
        executable='docking_node',
        name='docking_node',
        output='screen',
        parameters=[
            {'robot_id': robot_id},
            {'command': command},
        ]
    )

    return LaunchDescription([
        declare_robot_id,
        declare_command,
        docking_node,
    ])

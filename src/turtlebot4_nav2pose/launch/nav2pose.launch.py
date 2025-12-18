# my_nav_utils/launch/localization_with_initial_pose.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')  # rad

    # --- Launch arguments 선언 ---
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='/robot1',
        description='Robot namespace (e.g. /robot1)',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/home/rokey/turtlebot4_ws/maps/key_map.yaml',
        description='Full path to map yaml file',
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='/home/rokey/turtlebot4_ws/maps/local2.yaml',
        description='Full path to localization params file',
    )

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value='/home/rokey/turtlebot4_ws/maps/nav2_net2.yaml',
        description='Full path to Nav2 params yaml',
    )

    declare_initial_x = DeclareLaunchArgument(
        'initial_x',
        default_value='0.0',
        description='Initial pose X in map frame',
    )

    declare_initial_y = DeclareLaunchArgument(
        'initial_y',
        default_value='0.0',
        description='Initial pose Y in map frame',
    )

    declare_initial_yaw = DeclareLaunchArgument(
        'initial_yaw',
        default_value='0.0',
        description='Initial yaw (rad) in map frame',
    )

    # ===== 1) localization.launch.py include =====
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'localization.launch.py',
            ])
        ),
        launch_arguments={
            'namespace': namespace,
            'map': map_yaml,
            'params_file': params_file,
        }.items(),
    )

    # ===== 2) initial pose publisher =====
    # namespace가 걸린 상태에서 topic="initialpose" 로 퍼블리시 → /robot1/initialpose
    initial_pose_node = Node(
        package='turtlebot4_nav2pose',
        executable='init_pose_publisher',
        name='init_pose_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'x': initial_x,
            'y': initial_y,
            'yaw': initial_yaw,
            'frame_id': 'map',
            'topic': 'initialpose',
            'publish_delay': 3.0,  # AMCL가 완전히 뜨도록 약간 기다렸다가 쏘는 시간 (초)
        }],
    )

    # 혹시 localization이 뜨는 데 시간이 좀 더 필요하면 TimerAction 으로
    delayed_initial_pose = TimerAction(
        period=5.0,  # launch 시작 후 3초 뒤에 노드 시작
        actions=[initial_pose_node],
    )

    # ===== 3) nav2.launch.py include =====
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'nav2.launch.py',
            ])
        ),
        launch_arguments={
            'namespace': namespace,
            'params_file': nav2_params_file,
        }.items(),
    )

    delayed_nav2_launch = TimerAction(
        period=20.0,
        actions=[nav2_launch],
    )

    return LaunchDescription([
        declare_namespace,
        declare_map,
        declare_params,
        declare_nav2_params,
        declare_initial_x,
        declare_initial_y,
        declare_initial_yaw,
        localization_launch,
        delayed_initial_pose,
        delayed_nav2_launch,
    ])

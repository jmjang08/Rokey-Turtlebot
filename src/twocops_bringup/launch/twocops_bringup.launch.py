from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # ----- Launch arguments -----
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='1',
        description='Robot ID (1 or 3)',
    )

    declare_partner_robot_id = DeclareLaunchArgument(
        'partner_robot_id',
        default_value='3',
        description='Partner Robot ID (opposite of robot_id)',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/turtlebot4_ws/maps/key_map.yaml'),
        description='Full path to map yaml file',
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.expanduser('~/turtlebot4_ws/maps/local2.yaml'),
        description='Full path to localization params file',
    )

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.expanduser('~/turtlebot4_ws/maps/nav2_net2.yaml'),
        description='Full path to Nav2 params yaml',
    )

    declare_initial_pose_file = DeclareLaunchArgument(
        'initial_pose_file',
        default_value=os.path.expanduser('~/turtlebot4_ws/config/robot1_initial_pose.txt'),
        description='Text file containing "x y yaw" for initial pose',
    )

    declare_tracking_model_path = DeclareLaunchArgument(
        'tracking_model_path',
        default_value=os.path.expanduser('~/turtlebot4_ws/model/best.pt'),
        description='YOLO model path for turtlebot4_tracking',
    )

    # ================== 실제 액션 구성 ==================
    def launch_setup(context, *args, **kwargs):
        robot_id = LaunchConfiguration('robot_id').perform(context)
        partner_robot_id = LaunchConfiguration('partner_robot_id').perform(context)
        map_yaml = LaunchConfiguration('map').perform(context)
        params_file = LaunchConfiguration('params_file').perform(context)
        nav2_params_file = LaunchConfiguration('nav2_params_file').perform(context)
        initial_pose_file = LaunchConfiguration('initial_pose_file').perform(context)
        tracking_model_path = LaunchConfiguration('tracking_model_path').perform(context)

        ns_full = f'/robot{robot_id}'   # 예: /robot1
        ns_plain = f'robot{robot_id}'   # 예: robot1

        # ---- 1) initial pose 파일 읽기 ----
        try:
            with open(initial_pose_file, 'r') as f:
                content = f.read().strip()
            parts = content.split()
            if len(parts) != 3:
                raise ValueError(f"initial_pose_file must contain exactly 3 values, got: {content}")
            initial_x, initial_y, initial_yaw = parts
        except Exception as e:
            print(f"[twocops_bringup] Failed to read initial pose file: {initial_pose_file}")
            print(f"[twocops_bringup] Error: {e}")
            initial_x, initial_y, initial_yaw = '0.0', '0.0', '0.0'

        # ---- 2) Step1: DOCK 상태 체크 & 필요 시 UNDOCK ----
        dock_state_manager = Node(
            package='twocops_bringup',
            executable='dock_state_manager',
            name='dock_state_manager',
            output='screen',
            parameters=[{'robot_id': int(robot_id)}],
        )

        # ---- 3) Step2: prep_checker.sh 실행 (DockStateManager 종료 후) ----
        prep_checker_script = os.path.expanduser('~/turtlebot4_ws/prep_checker.sh')

        prep_checker = ExecuteProcess(
            cmd=[prep_checker_script, '--n', str(robot_id)],
            output='screen',
        )

        # ---- 4) Step3: nav2pose.launch.py 실행 ----
        nav2pose_pkg_share = get_package_share_directory('turtlebot4_nav2pose')
        nav2pose_launch_path = os.path.join(
            nav2pose_pkg_share, 'launch', 'nav2pose.launch.py'
        )

        nav2pose_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2pose_launch_path),
            launch_arguments={
                'namespace': ns_full,
                'map': map_yaml,
                'params_file': params_file,
                'nav2_params_file': nav2_params_file,
                'initial_x': initial_x,
                'initial_y': initial_y,
                'initial_yaw': initial_yaw,
            }.items(),
        )

        # ---- 5) Nav2ReadyWaiter: Nav2 액션 서버 준비될 때까지 대기 ----
        nav2_ready_waiter_node = Node(
            package='twocops_bringup',
            executable='nav2_ready_waiter',
            name=f'nav2_ready_waiter_robot{robot_id}',
            output='screen',
            parameters=[
                {'robot_id': int(robot_id)},
                {'timeout': 180.0},
            ],
        )

        # ---- 6) Step4: tracking(yolo_move_robot.launch.py) 실행 ----
        tracking_pkg_share = get_package_share_directory('turtlebot4_tracking')
        tracking_launch_path = os.path.join(
            tracking_pkg_share, 'launch', 'yolo_move_robot.launch.py'
        )

        tracking_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracking_launch_path),
            launch_arguments={
                'namespace': ns_plain,
                'model_path': tracking_model_path,
                'rgb_topic': f'{ns_full}/oakd/rgb/image_raw/compressed',
                'depth_topic': f'{ns_full}/oakd/stereo/image_raw',
                'cam_info_topic': f'{ns_full}/oakd/rgb/camera_info',
            }.items(),
        )

        # ---- 7) Step5: TransitionManager 실행 ----
        transition_manager_node = Node(
            package='twocops_bringup',
            executable='transition_manager',
            name=f'transition_manager_robot{robot_id}',
            output='screen',
            parameters=[
                {'robot_id': int(robot_id)},
                {'partner_robot_id': int(partner_robot_id)},
            ],
        )

        # ---- 이벤트 체인 구성 ----
        # 1) DockStateManager 종료 → prep_checker 실행
        event_after_dock = RegisterEventHandler(
            OnProcessExit(
                target_action=dock_state_manager,
                on_exit=[prep_checker],
            )
        )

        # 2) prep_checker 종료 → nav2pose.launch + nav2_ready_waiter 실행
        event_after_prep = RegisterEventHandler(
            OnProcessExit(
                target_action=prep_checker,
                on_exit=[nav2pose_launch, nav2_ready_waiter_node],
            )
        )

        # 3) nav2_ready_waiter 종료(= Nav2 준비 완료) → tracking + transition_manager 실행
        event_after_nav2_ready = RegisterEventHandler(
            OnProcessExit(
                target_action=nav2_ready_waiter_node,
                on_exit=[tracking_launch, transition_manager_node],
            )
        )

        return [
            dock_state_manager,
            event_after_dock,
            event_after_prep,
            event_after_nav2_ready,
        ]

    opaque_setup = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        declare_robot_id,
        declare_partner_robot_id,
        declare_map,
        declare_params,
        declare_nav2_params,
        declare_initial_pose_file,
        declare_tracking_model_path,
        opaque_setup,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    model_path = LaunchConfiguration('model_path')

    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    cam_info_topic = LaunchConfiguration('cam_info_topic')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='robot3')
    declare_model_path = DeclareLaunchArgument('model_path', default_value='/home/rokey/turtlebot4_ws/model/best.pt')
    declare_rgb_topic = DeclareLaunchArgument('rgb_topic', default_value='oakd/rgb/image_raw/compressed')
    declare_depth_topic = DeclareLaunchArgument('depth_topic', default_value='oakd/stereo/image_raw')
    declare_cam_info_topic = DeclareLaunchArgument('cam_info_topic', default_value='oakd/rgb/camera_info')

    # YOLO + Depth Fusion Node
    detection_node = Node(
        package='turtlebot4_tracking',
        executable='detection',
        name='detection',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'model_path': model_path,
                'rgb_topic': rgb_topic,
                'depth_topic': depth_topic,
                'cam_info_topic': cam_info_topic
            }
        ]
    )

    # MoveRobot Node
    tracking_node = Node(
        package='turtlebot4_tracking',
        executable='tracking',
        name='tracking',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    target_calculator = Node(
        package='turtlebot4_tracking',
        executable='target_calculator',
        name='target_calculator',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        declare_namespace,
        declare_model_path,
        declare_rgb_topic,
        declare_depth_topic,
        declare_cam_info_topic,
        detection_node,
        tracking_node,
        target_calculator,
    ])

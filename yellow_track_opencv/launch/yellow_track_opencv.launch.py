import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    web_show_opencv = os.getenv('WEB_SHOW_OPENCV')
    print("WEB_SHOW_OPENCV is", web_show_opencv)

    # web可视化节点，订阅 /yellow_track_vis
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/yellow_track_vis',
            'websocket_image_type':  'mjpeg',
            'websocket_only_show_image': 'True',
        }.items()
    )

    yellow_track_node = Node(
        package='yellow_track_opencv',
        executable='yellow_track_opencv',
        output='screen',
        parameters=[{
            # HSV黄色阈值，比赛现场可直接修改
            'h_min':      LaunchConfiguration('h_min'),
            'h_max':      LaunchConfiguration('h_max'),
            's_min':      LaunchConfiguration('s_min'),
            's_max':      LaunchConfiguration('s_max'),
            'v_min':      LaunchConfiguration('v_min'),
            'v_max':      LaunchConfiguration('v_max'),
            # ROI区域
            'roi_top':    LaunchConfiguration('roi_top'),
            'roi_bottom': LaunchConfiguration('roi_bottom'),
            # 目标边线x坐标
            'target_x':   LaunchConfiguration('target_x'),
            # 找不到边线时的error
            'lost_error': LaunchConfiguration('lost_error'),
        }],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    launch_args = [
        DeclareLaunchArgument('h_min',      default_value='0'),
        DeclareLaunchArgument('h_max',      default_value='35'),
        DeclareLaunchArgument('s_min',      default_value='15'),
        DeclareLaunchArgument('s_max',      default_value='78'),
        DeclareLaunchArgument('v_min',      default_value='121'),
        DeclareLaunchArgument('v_max',      default_value='220'),
        DeclareLaunchArgument('roi_top',    default_value='0.35'),
        DeclareLaunchArgument('roi_bottom', default_value='0.65'),
        DeclareLaunchArgument('target_x',   default_value='600.0'),
        DeclareLaunchArgument('lost_error', default_value='80.0'),
    ]

    if web_show_opencv == 'TRUE':
        return LaunchDescription(launch_args + [
            yellow_track_node,
            web_node,
        ])
    else:
        return LaunchDescription(launch_args + [
            yellow_track_node,
        ])
        
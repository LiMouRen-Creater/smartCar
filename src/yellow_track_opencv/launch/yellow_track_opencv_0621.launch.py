# -*- coding: utf-8 -*-
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

    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/track_center_vis',
            'websocket_image_type':  'mjpeg',
            'websocket_only_show_image': 'True',
        }.items()
    )

    yellow_track_node_0621 = Node(
        package='yellow_track_opencv',       # TODO: 改成实际包名
        executable='yellow_track_opencv_0621',    # TODO: 改成setup.py里注册的可执行文件名
        output='screen',
        parameters=[{
            # HSV青绿色阈值
            'h_min': LaunchConfiguration('h_min'),
            'h_max': LaunchConfiguration('h_max'),
            's_min': LaunchConfiguration('s_min'),
            's_max': LaunchConfiguration('s_max'),
            'v_min': LaunchConfiguration('v_min'),
            'v_max': LaunchConfiguration('v_max'),
            'denoise_ksize': LaunchConfiguration('denoise_ksize'),
            # 主ROI范围
            'roi_top':    LaunchConfiguration('roi_top'),
            'roi_bottom': LaunchConfiguration('roi_bottom'),
            # 孤岛检测ROI范围
            'island_roi_top':    LaunchConfiguration('island_roi_top'),
            'island_roi_bottom': LaunchConfiguration('island_roi_bottom'),
            # 最长黑列法参数
            'plateau_ratio': LaunchConfiguration('plateau_ratio'),
            # 目标位置(按方向区分)
            'target_x_default': LaunchConfiguration('target_x_default'),
            'target_x_cw':       LaunchConfiguration('target_x_cw'),
            'target_x_ccw':       LaunchConfiguration('target_x_ccw'),
            # 孤岛/盲转参数
            'island_area_thresh': LaunchConfiguration('island_area_thresh'),
            'shrink_thresh':      LaunchConfiguration('shrink_thresh'),
            'confirm_frames':     LaunchConfiguration('confirm_frames'),
            'amplify':            LaunchConfiguration('amplify'),
            # 孤岛预期方位(诊断用)
            'island_side_cw':  LaunchConfiguration('island_side_cw'),
            'island_side_ccw': LaunchConfiguration('island_side_ccw'),
            # 赛道宽度记忆
            'width_ema_alpha': LaunchConfiguration('width_ema_alpha'),
            # 丢线error
            'lost_error': LaunchConfiguration('lost_error'),
        }],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    launch_args = [
        # ============ HSV青绿色阈值 ============
        DeclareLaunchArgument('h_min', default_value='75'),
        DeclareLaunchArgument('h_max', default_value='140'),
        DeclareLaunchArgument('s_min', default_value='0'),
        DeclareLaunchArgument('s_max', default_value='255'),
        DeclareLaunchArgument('v_min', default_value='0'),
        DeclareLaunchArgument('v_max', default_value='255'),
        DeclareLaunchArgument('denoise_ksize', default_value='3'),

        # ============ 主ROI范围 ============
        DeclareLaunchArgument('roi_top',    default_value='0.40'),
        DeclareLaunchArgument('roi_bottom', default_value='0.72'),

        # ============ 孤岛检测ROI范围 ============
        DeclareLaunchArgument('island_roi_top',    default_value='0.33'),
        DeclareLaunchArgument('island_roi_bottom', default_value='0.60'),

        # ============ 最长黑列法参数 ============
        DeclareLaunchArgument('plateau_ratio', default_value='0.98'),

        # ============ 目标位置(按方向区分) ============
        DeclareLaunchArgument('target_x_default', default_value='320.0'),
        DeclareLaunchArgument('target_x_cw',       default_value='320.0'),
        DeclareLaunchArgument('target_x_ccw',      default_value='320.0'),

        # ============ 孤岛/盲转参数 ============
        DeclareLaunchArgument('island_area_thresh', default_value='5000'),
        DeclareLaunchArgument('shrink_thresh',      default_value='5000'),
        DeclareLaunchArgument('confirm_frames',     default_value='3'),
        DeclareLaunchArgument('amplify',            default_value='1.5'),  # 建议保守起步，比代码默认值3.0更小

        # ============ 孤岛预期方位(目前只是诊断日志用) ============
        DeclareLaunchArgument('island_side_cw',  default_value='left'),
        DeclareLaunchArgument('island_side_ccw', default_value='right'),

        # ============ 赛道宽度记忆 ============
        DeclareLaunchArgument('width_ema_alpha', default_value='0.1'),

        # ============ 丢线error ============
        DeclareLaunchArgument('lost_error', default_value='100.0'),
    ]

    if web_show_opencv == 'TRUE':
        return LaunchDescription(launch_args + [
            yellow_track_node_0621,
            web_node,
        ])
    else:
        return LaunchDescription(launch_args + [
            yellow_track_node_0621,
        ])
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
            # HSV黄色阈值
            'h_min': LaunchConfiguration('h_min'),
            'h_max': LaunchConfiguration('h_max'),
            's_min': LaunchConfiguration('s_min'),
            's_max': LaunchConfiguration('s_max'),
            'v_min': LaunchConfiguration('v_min'),
            'v_max': LaunchConfiguration('v_max'),
            # 近处ROI范围
            'roi_top':    LaunchConfiguration('roi_top'),
            'roi_bottom': LaunchConfiguration('roi_bottom'),
            # 远处ROI范围
            'roi_far_top':    LaunchConfiguration('roi_far_top'),
            'roi_far_bottom': LaunchConfiguration('roi_far_bottom'),
            # 近处采样和评估
            'n_samples':            LaunchConfiguration('n_samples'),
            'outlier_thresh_ratio': LaunchConfiguration('outlier_thresh_ratio'),
            'eval_offset':          LaunchConfiguration('eval_offset'),
            # 远处采样和评估
            'n_samples_far':            LaunchConfiguration('n_samples_far'),
            'outlier_thresh_ratio_far': LaunchConfiguration('outlier_thresh_ratio_far'),
            'eval_offset_far':          LaunchConfiguration('eval_offset_far'),
            # 近处目标边线x（逆时针=600，顺时针=40）
            'target_x_near': LaunchConfiguration('target_x_near'),
            # 远处目标边线x（逆时针=450，顺时针=90）
            'target_x_far':  LaunchConfiguration('target_x_far'),
            # 丢线error
            'lost_error': LaunchConfiguration('lost_error'),
            # 加权融合
            'weight_near':   LaunchConfiguration('weight_near'),
            'weight_far':    LaunchConfiguration('weight_far'),
            'max_far_error': LaunchConfiguration('max_far_error'),
        }],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    launch_args = [
        # HSV黄色阈值
        DeclareLaunchArgument('h_min', default_value='0'),
        DeclareLaunchArgument('h_max', default_value='35'),
        DeclareLaunchArgument('s_min', default_value='12'),
        DeclareLaunchArgument('s_max', default_value='80'),
        DeclareLaunchArgument('v_min', default_value='121'),
        DeclareLaunchArgument('v_max', default_value='230'),
        # 近处ROI范围
        DeclareLaunchArgument('roi_top',    default_value='0.35'),
        DeclareLaunchArgument('roi_bottom', default_value='0.60'),
        # 远处ROI范围
        DeclareLaunchArgument('roi_far_top',    default_value='0.30'),
        DeclareLaunchArgument('roi_far_bottom', default_value='0.50'),
        # 近处采样和评估
        DeclareLaunchArgument('n_samples',            default_value='15'),
        DeclareLaunchArgument('outlier_thresh_ratio', default_value='0.15'),
        DeclareLaunchArgument('eval_offset',          default_value='36'),
        # 远处采样和评估
        DeclareLaunchArgument('n_samples_far',            default_value='15'),
        DeclareLaunchArgument('outlier_thresh_ratio_far', default_value='0.15'),
        DeclareLaunchArgument('eval_offset_far',          default_value='5'),
        # 近处目标边线x（逆时针=600，顺时针=40）
        DeclareLaunchArgument('target_x_near', default_value='600.0'),
        # 远处目标边线x（逆时针=450，顺时针=90）
        DeclareLaunchArgument('target_x_far',  default_value='490.0'),
        # 丢线error
        DeclareLaunchArgument('lost_error', default_value='80.0'),
        # 加权融合
        DeclareLaunchArgument('weight_near',   default_value='0.7'),
        DeclareLaunchArgument('weight_far',    default_value='0.3'),
        DeclareLaunchArgument('max_far_error', default_value='100.0'),
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
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动完整图像标注显示流程

启动USB摄像头、YOLO障碍物检测、ResNet黑线检测、
底盘、二维码、视觉语言模型、rosbridge和标注节点。

订阅话题：
  - 无（本文件仅为启动文件）

发布话题：
  - /image_annotated (CompressedImage): 带标注的JPEG图像
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    """生成启动描述，包含摄像头、检测、底盘、通信和标注节点"""

    # 声明启动参数
    launch_args = [
        DeclareLaunchArgument('device', default_value='/dev/video0',
                               description='USB摄像头设备路径'),
    ]

    # 1. USB摄像头节点（零拷贝模式）
    usb_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_usb_cam')
            + '/launch/hobot_usb_cam.launch.py'),
        launch_arguments={
            'usb_image_width': '640',
            'usb_image_height': '480',
            'usb_zero_copy': 'True',
            'usb_video_device': LaunchConfiguration('device'),
        }.items())

    # 2. OpenCV黄色车道线检测节点（HSV黄色提取 + 轮廓中心检测）
    yellow_track_node = Node(
        package='yellow_track_opencv',
        executable='yellow_track_opencv',
        name='yellow_track_opencv',
        output='screen',
        parameters=[{
            'hsv_h_min': 15, 'hsv_h_max': 35,
            'hsv_s_min': 80, 'hsv_s_max': 255,
            'hsv_v_min': 80, 'hsv_v_max': 255,
            'roi_top': 240, 'roi_bottom': 480,
            'roi_left': 0, 'roi_right': 640,
            'min_contour_area': 200.0,
            'morph_kernel_size': 5,
            'detection_method': 0,
            'sub_img_topic': '/hbmem_img',
            'pub_topic': '/yellow_track_center',
            'debug_output': False,
            'visualize': False,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    )

    # 3. NV12解码节点（JPEG转NV12到共享内存）
    nv12_decode_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_codec')
            + '/launch/hobot_codec_decode.launch.py'),
        launch_arguments={
            'codec_channel': '1',
            'codec_in_format': 'jpeg',
            'codec_out_format': 'nv12',
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/nv12_img',
            'src_image_width': '640',
            'src_image_height': '480',
        }.items())

    # 3. YOLO障碍物检测
    racing_obstacle_detection_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('racing_obstacle_detection_yolo')
            + '/launch/racing_obstacle_detection_yolo.launch.py'))

    # 4. ResNet黑线检测
    racing_track_detection_resnet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('racing_track_detection_resnet')
            + '/launch/racing_track_detection_resnet.launch.py'))

    # 5. 底盘驱动
    origincar_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('origincar_base')
            + '/launch/origincar_bringup.launch.py'))

    # 6. 视觉语言模型
    vision_language_model = Node(
        package='vision_language_model',
        executable='vision_language_model',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'])

    # 7. 图像转模型
    img_to_model = Node(
        package='img_to_model',
        executable='img_to_model',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'])

    # 8. 二维码检测
    qrcode = Node(
        package='qrcode',
        executable='qrcode',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'])

    # 9. rosbridge通信（上位机通过WebSocket查看画面）
    rosbridge_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen')

    # 10. image_annotator标注节点
    annotator = Node(
        package='image_annotator',
        executable='image_annotator',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'])

    return LaunchDescription(launch_args + [
        usb_node,
        yellow_track_node,
        nv12_decode_node,
        racing_obstacle_detection_yolo,
        racing_track_detection_resnet,
        origincar_base,
        vision_language_model,
        img_to_model,
        qrcode,
        rosbridge_node,
        annotator,
    ])
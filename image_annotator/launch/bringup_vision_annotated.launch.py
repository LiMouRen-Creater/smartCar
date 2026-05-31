#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bringup_vision_annotated.launch.py  — 视觉标注完整启动文件

【图像链路】
  USB Camera
    │  (usb_zero_copy, JPEG shared_mem)
    ▼
  /hbmem_img      ← image_annotator 标注节点从此获取原图
    │  (hobot_codec decode: JPEG→NV12)
    ▼
  /nv12_img       ← YOLO/ResNet/OpenCV三个检测节点统一从此获取NV12图像
    │
    ├── racing_obstacle_detection_yolo
    │     (is_shared_mem_sub, sub_img_topic:=/nv12_img)
    │     → /racing_obstacle_detection
    │
    ├── racing_track_detection_resnet
    │     (sub_img_topic:=/nv12_img)
    │     → /racing_track_center_detection
    │
    └── yellow_track_opencv
          (sub_img_topic:=/nv12_img)
          → /yellow_track_center
    │
    └── image_annotator (订阅 /hbmem_img 原图 + 三个检测结果)
          → /image_annotated (320×240, CompressedImage JPEG)

【启动顺序】
  1. USB 摄像头（hobot_usb_cam）
  2. NV12 解码器（hobot_codec decode）
  3. 算法检测节点（YOLO / ResNet / OpenCV）
  4. 底盘、二维码、VLM、img_to_model
  5. rosbridge_server（WebSocket转发）
  6. image_annotator 标注节点

【使用方式】
  ros2 launch image_annotator bringup_vision_annotated.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix


def generate_launch_description():
    """生成完整视觉标注启动描述"""

    # ===== 启动参数声明 =====
    launch_args = [
        DeclareLaunchArgument(
            'device', default_value='/dev/video0',
            description='USB摄像头设备路径'),
    ]

    # ================================================================
    # 1. USB 摄像头节点
    #    输入: 无
    #    输出: /hbmem_img (JPEG, shared_mem, 640×480)
    #    模式: zero_copy = True
    # ================================================================
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

    # ================================================================
    # 2. hobot_codec 解码节点（JPEG → NV12）
    #    输入: /hbmem_img (JPEG, shared_mem)
    #    输出: /nv12_img (NV12, shared_mem)
    #    说明: 将摄像头JPEG解码为NV12供算法节点使用
    # ================================================================
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

    # ================================================================
    # 3a. YOLO 障碍物检测节点
    #     输入: /nv12_img (NV12, shared_mem)
    #     输出: /racing_obstacle_detection (PerceptionTargets)
    #     参数: is_shared_mem_sub=True, sub_img_topic:=/nv12_img
    # ================================================================
    racing_obstacle_detection_yolo_node = Node(
        package='racing_obstacle_detection_yolo',
        executable='racing_obstacle_detection_yolo',
        output='screen',
        parameters=[{
            'is_shared_mem_sub': True,
            'sub_img_topic': '/nv12_img',
            'config_file': os.path.join(
                get_package_prefix('racing_obstacle_detection_yolo'),
                'lib', 'racing_obstacle_detection_yolo', 'config', 'yolov5sconfig.json'),
        }],
        arguments=['--ros-args', '--log-level', 'error'],
    )

    # ================================================================
    # 3b. ResNet 赛道/黑线检测节点
    #     输入: /nv12_img (NV12, shared_mem)
    #     输出: /racing_track_center_detection (PerceptionTargets)
    #     参数: sub_img_topic:=/nv12_img
    # ================================================================
    racing_track_detection_resnet_node = Node(
        package='racing_track_detection_resnet',
        executable='racing_track_detection_resnet',
        output='screen',
        parameters=[{
            'sub_img_topic': '/nv12_img',
            'model_path': os.path.join(
                get_package_prefix('racing_track_detection_resnet'),
                'lib', 'racing_track_detection_resnet', 'config', 'converted_model.bin'),
        }],
        arguments=['--ros-args', '--log-level', 'error'],
    )

    # ================================================================
    # 3c. OpenCV 黄色车道线检测节点
    #     输入: /nv12_img (NV12, shared_mem) ← 关键修改：原为 /hbmem_img
    #     输出: /yellow_track_center (PerceptionTargets)
    # ================================================================
    yellow_track_node = Node(
        package='yellow_track_opencv',
        executable='yellow_track_opencv',
        name='yellow_track_opencv',
        output='screen',
        parameters=[{
            # HSV参数
            'hsv_h_min': 15, 'hsv_h_max': 35,
            'hsv_s_min': 80, 'hsv_s_max': 255,
            'hsv_v_min': 80, 'hsv_v_max': 255,
            # ROI参数
            'roi_top': 240, 'roi_bottom': 480,
            'roi_left': 0, 'roi_right': 640,
            # 轮廓处理参数
            'min_contour_area': 200.0,
            'morph_kernel_size': 5,
            'detection_method': 0,
            # ★★★ 关键修改：订阅解码后的 NV12 话题，不再直接订阅 /hbmem_img ★★★
            'sub_img_topic': '/nv12_img',
            # 发布话题
            'pub_topic': '/yellow_track_center',
            # 调试参数
            'debug_output': False,
            'visualize': False,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    )

    # ================================================================
    # 4. 底盘驱动节点（origincar_base）
    #    输入: /cmd_vel (Twist)
    #    输出: 串口控制信号
    # ================================================================
    origincar_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('origincar_base')
            + '/launch/origincar_bringup.launch.py'))

    # ================================================================
    # 5. 视觉语言模型节点（VLM）
    #    输入: /hbmem_img 等
    #    输出: 语音/文字结果
    # ================================================================
    vision_language_model = Node(
        package='vision_language_model',
        executable='vision_language_model',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'])

    # ================================================================
    # 6. 图像转模型节点（img_to_model）
    #    输入: /hbmem_img (NV12, shared_mem)
    #    输出: 模型推理结果
    # ================================================================
    img_to_model = Node(
        package='img_to_model',
        executable='img_to_model',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'])

    # ================================================================
    # 7. 二维码检测节点
    #    输入: /hbmem_img (NV12, shared_mem)
    #    输出: /qrcode_detection
    # ================================================================
    qrcode = Node(
        package='qrcode',
        executable='qrcode',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'])

    # ================================================================
    # 8. rosbridge WebSocket 服务器
    #    用途: 上位机通过 WebSocket 查看 ROS 话题
    #    方式: 使用 Node 启动 rosbridge_server 的 websocket 主节点
    # ================================================================
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '',
            'authenticate': False,
            'retry_startup_delay': 5.0,
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ================================================================
    # 9. 图像标注节点（image_annotator）
    #    输入: /hbmem_img (JPEG, shared_mem) ← 使用原始JPEG原图
    #         /racing_obstacle_detection (YOLO检测结果)
    #         /racing_track_center_detection (ResNet黑线中心)
    #         /yellow_track_center (OpenCV黄色车道线中心)
    #    输出: /image_annotated (CompressedImage JPEG, 320×240)
    #
    #    参数说明（需要与 image_annotator.py 节点源码参数名保持一致）：
    #      raw_image_topic:      输入原图话题
    #      yellow_center_topic:  黄色车道线中心话题
    #      output_topic:         标注图像输出话题
    #      resize_scale:         输出缩放比例（0.5 表示 50%）
    #      output_width:         输出图像宽度（320）
    #      output_height:        输出图像高度（240）
    # ================================================================
    annotator = Node(
        package='image_annotator',
        executable='image_annotator',
        output='screen',
        parameters=[{
            # 输入话题（需要与节点源码参数名保持一致）
            'raw_image_topic': '/hbmem_img',
            'yellow_center_topic': '/yellow_track_center',
            # 输出话题
            'output_topic': '/image_annotated',
            # 输出尺寸参数
            'resize_scale': 0.5,
            'output_width': 320,
            'output_height': 240,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    )

    # ===== 返回 LaunchDescription =====
    # 启动顺序：USB摄像头 → NV12解码 → 算法检测 → 其他节点 → 标注节点
    return LaunchDescription(launch_args + [
        usb_node,           # 1. USB摄像头
        nv12_decode_node,   # 2. hobot_codec 解码（JPEG→NV12）
        # 3. 算法检测节点（统一订阅 /nv12_img）
        racing_obstacle_detection_yolo_node,   # 3a. YOLO障碍物检测
        racing_track_detection_resnet_node,    # 3b. ResNet黑线检测
        yellow_track_node,                     # 3c. OpenCV黄色车道线
        # 4. 其他功能节点
        origincar_base,     # 底盘驱动
        qrcode,             # 二维码检测
        vision_language_model,  # 视觉语言模型
        img_to_model,       # 图像转模型
        rosbridge_node,     # rosbridge WebSocket
        annotator,          # 9. image_annotator 标注节点（最后启动）
    ])
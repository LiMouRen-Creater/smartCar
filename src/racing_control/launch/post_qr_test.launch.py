#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=====================================================================
post_qr_test.launch.py  - 后QR码控制测试启动文件
=====================================================================

【用途】
调试扫描二维码之后的后QR码状态机控制：
  Phase1: 向左行驶1.5米 → Phase2: 右转对准黄色道路
  Phase3: 黄色车道线巡线绕圈（含锥桶避障）
  → 黑线重现退出

【启动方式】
  ros2 launch racing_control post_qr_test.launch.py

【可覆盖参数】
  后QR参数：
    post_qr_forward_speed:=0.5       Phase1左转速度(m/s)
    post_qr_forward_time_ms:=3000    Phase1左转时间(ms) ≈1.5米
    post_qr_forward_angular_z:=0.3   Phase1左转角速度(rad/s)
    post_qr_turn_speed:=0.3          Phase2右转角速度(rad/s)
    post_qr_turn_timeout_ms:=3000    Phase2右转超时(ms)
    post_qr_yellow_area_threshold:=0.15  黄色占比阈值
    post_qr_follow_speed:=0.4        Phase3黄色巡线速度(m/s)
    post_qr_follow_kp:=0.005         Phase3黄色巡线比例系数

  OpenCV黄色检测参数：
    hsv_h_min:=15  hsv_h_max:=35  ...

【测试时实时查看】
  ros2 topic echo /cmd_vel               # 查看控制指令
  ros2 topic echo /yellow_track_center   # 查看黄色检测结果
  ros2 topic echo /racing_track_center_detection  # 查看黑线

  可视化（如果yellow_track_opencv开启visualize=true）：
  ros2 run rqt_image_view rqt_image_view /yellow_track_debug

=====================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ================================================================
    # 声明后QR控制参数（与post_qr_control.cpp一致）
    # ================================================================
    post_qr_args = [
        DeclareLaunchArgument(
            'post_qr_forward_speed', default_value='0.5',
            description='Phase1左转速度(m/s)'),
        DeclareLaunchArgument(
            'post_qr_forward_time_ms', default_value='3000',
            description='Phase1左转时间(ms) 约1.5米'),
        DeclareLaunchArgument(
            'post_qr_forward_angular_z', default_value='0.3',
            description='Phase1左转角速度(rad/s)'),
        DeclareLaunchArgument(
            'post_qr_turn_speed', default_value='0.3',
            description='Phase2右转角速度(rad/s)'),
        DeclareLaunchArgument(
            'post_qr_turn_timeout_ms', default_value='3000',
            description='Phase2右转超时(ms)'),
        DeclareLaunchArgument(
            'post_qr_yellow_area_threshold', default_value='0.15',
            description='黄色面积占比阈值≥此值认为进入通道'),
        DeclareLaunchArgument(
            'post_qr_follow_speed', default_value='0.4',
            description='Phase3黄色巡线速度(m/s)'),
        DeclareLaunchArgument(
            'post_qr_follow_kp', default_value='0.005',
            description='Phase3黄色巡线比例系数'),
        # 锥桶避障参数
        DeclareLaunchArgument(
            'avoid_x', default_value='0.8',
            description='锥桶避障速度(m/s)'),
        DeclareLaunchArgument(
            'avoid_kp', default_value='0.0035',
            description='锥桶避障比例系数'),
        DeclareLaunchArgument(
            'end_y', default_value='190',
            description='锥桶触发避障的底部y阈值'),
    ]

    # ================================================================
    # 1. USB相机节点（图像来源）
    # ================================================================
    usb_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'),
        launch_arguments={
            'usb_image_width': '640',
            'usb_image_height': '480',
            'usb_zero_copy': 'True',
            'usb_video_device': LaunchConfiguration('device', default='/dev/video0'),
        }.items(),
    )

    # ================================================================
    # 2. NV12解码节点（JPEG→NV12共享内存）
    # ================================================================
    nv12_decode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
        launch_arguments={
            'codec_channel': '1',
            'codec_in_format': 'jpeg',
            'codec_out_format': 'nv12',
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/nv12_img',
        }.items(),
    )

    # ================================================================
    # 3. 车辆底盘节点（串口通信）
    # ================================================================
    origincar_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('origincar_base') + '/launch/base_serial.launch.py'),
        launch_arguments={'akmcar': 'false'}.items(),
    )

    # ================================================================
    # 4. OpenCV黄色车道线检测节点（后QR黄道巡线的核心输入）
    #    输出话题：/yellow_track_center → 同时转发到 /image_yellow_track_test
    # ================================================================
    yellow_track_opencv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('yellow_track_opencv') + '/launch/yellow_track_opencv.launch.py'),
        launch_arguments={
            'hsv_h_min': LaunchConfiguration('hsv_h_min', default='15'),
            'hsv_h_max': LaunchConfiguration('hsv_h_max', default='35'),
            'hsv_s_min': LaunchConfiguration('hsv_s_min', default='80'),
            'hsv_s_max': LaunchConfiguration('hsv_s_max', default='255'),
            'hsv_v_min': LaunchConfiguration('hsv_v_min', default='80'),
            'hsv_v_max': LaunchConfiguration('hsv_v_max', default='255'),
            'roi_top': LaunchConfiguration('roi_top', default='240'),
            'roi_bottom': LaunchConfiguration('roi_bottom', default='480'),
            'visualize': LaunchConfiguration('visualize', default='true'),
            'debug_output': LaunchConfiguration('debug_output', default='true'),
            'pub_topic': '/yellow_track_center',
        }.items(),
    )

    # ================================================================
    # 5. 话题转发节点（/yellow_track_center → /image_yellow_track_test）
    #    便于调试时单独观察黄色车道线检测结果
    # ================================================================
    topic_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='yellow_track_relay',
        output='screen',
        arguments=[
            '/yellow_track_center',
            '/image_yellow_track_test',
            '--qos-profile', 'sensor_data',
        ],
    )

    # ================================================================
    # 6. ResNet黑线检测节点（用于检测黑线重现，判断离开黄道的时机）
    # ================================================================
    racing_track_detection_resnet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('racing_track_detection_resnet') +
            '/launch/racing_track_detection_resnet.launch.py'),
    )

    # ================================================================
    # 7. YOLO锥桶障碍物检测节点（后QR黄道巡线绕圈时避障）
    # ================================================================
    racing_obstacle_detection_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('racing_obstacle_detection_yolo') +
            '/launch/racing_obstacle_detection_yolo.launch.py'),
    )

    # ================================================================
    # 8. QR码解码节点（触发后QR状态机的信号源）
    # ================================================================
    qrcode_node = Node(
        package='qrcode',
        executable='qrcode',
        name='qrcode',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # ================================================================
    # 9. 后QR码控制测试节点（post_qr_control）
    #    接收QR码触发，执行PostQR状态机控制
    # ================================================================
    post_qr_control_node = Node(
        package='racing_control',
        executable='post_qr_control',
        name='post_qr_control',
        output='screen',
        parameters=[{
            'post_qr_forward_speed': LaunchConfiguration('post_qr_forward_speed'),
            'post_qr_forward_time_ms': LaunchConfiguration('post_qr_forward_time_ms'),
            'post_qr_forward_angular_z': LaunchConfiguration('post_qr_forward_angular_z'),
            'post_qr_turn_speed': LaunchConfiguration('post_qr_turn_speed'),
            'post_qr_turn_timeout_ms': LaunchConfiguration('post_qr_turn_timeout_ms'),
            'post_qr_yellow_area_threshold': LaunchConfiguration('post_qr_yellow_area_threshold'),
            'post_qr_follow_speed': LaunchConfiguration('post_qr_follow_speed'),
            'post_qr_follow_kp': LaunchConfiguration('post_qr_follow_kp'),
            'avoid_x': LaunchConfiguration('avoid_x'),
            'avoid_kp': LaunchConfiguration('avoid_kp'),
            'end_y': LaunchConfiguration('end_y'),
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    )

    # ================================================================
    # 【预留节点 - 图文标识牌识别】
    #
    # 以下节点的代码已存在，但当前测试后QR控制时不需要启用。
    # 等你完成图文标识牌识别的调试后，取消注释即可启用。
    #
    # 注意：图片话题需要与解码话题匹配，如 /hbmem_img 或 /nv12_img
    # ================================================================
    #
    # 图文标识牌 → 中间话题的转换节点
    # img_to_model_node = Node(
    #     package='img_to_model',
    #     executable='img_to_model',
    #     name='img_to_model',
    #     output='screen',
    #     arguments=['--ros-args', '--log-level', 'info'],
    # )
    #
    # 大模型视觉语言理解（VLM）节点
    # vision_language_model_node = Node(
    #     package='vision_language_model',
    #     executable='vision_language_model',
    #     name='vision_language_model',
    #     output='screen',
    #     arguments=['--ros-args', '--log-level', 'info'],
    # )

    # ================================================================
    # 【可选节点 - rosbridge WebSocket】
    # 如需在网页端查看图像，取消注释
    # ================================================================
    # rosbridge_node = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
    #     output='screen',
    # )

    # ================================================================
    # 组装启动描述
    # ================================================================
    return LaunchDescription(post_qr_args + [
        # ---------- 节点按启动顺序排列 ----------
        # 1. 底层硬件
        usb_cam,                                    # USB相机
        nv12_decode,                                # 解码NV12
        origincar_base,                             # 车辆底盘串口

        # 2. 感知层
        racing_track_detection_resnet,              # 黑线检测（ResNet）
        racing_obstacle_detection_yolo,             # 锥桶检测（YOLO）
        yellow_track_opencv,                        # 黄色车道线检测（OpenCV）
        qrcode_node,                                # QR码解码

        # 3. 话题转发
        topic_relay_node,    # /yellow_track_center → /image_yellow_track_test

        # 4. 控制层
        post_qr_control_node,                       # 后QR码控制

        # 5. 【预留】图文标识牌 + 大模型视觉语言理解
        # （取消注释即可启用）
        # img_to_model_node,
        # vision_language_model_node,

        # 6. 【可选】WebSocket（网页端查看）
        # rosbridge_node,
    ])
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""黄色车道线检测启动文件 - 带完整调试参数接口

启动方式：
  ros2 launch yellow_track_opencv yellow_track_opencv.launch.py

可通过命令行覆盖所有参数，方便调试：
  ros2 launch yellow_track_opencv yellow_track_opencv.launch.py \
      hsv_h_min:=20 hsv_h_max:=30 \
      roi_top:=200 roi_bottom:=460 \
      detection_method:=1 visualize:=true

运行时也可以通过 ros2 param set 动态调整：
  ros2 param set /yellow_track_opencv hsv_h_min 20
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成启动描述，暴露全部可调试参数"""

    # ============================================================
    # HSV颜色阈值参数（黄色推荐H:15~35, S:80~255, V:80~255）
    # ============================================================
    hsv_h_min_arg = DeclareLaunchArgument(
        'hsv_h_min', default_value='15',
        description='HSV色调最小值（黄色车道线推荐15~35）')

    hsv_h_max_arg = DeclareLaunchArgument(
        'hsv_h_max', default_value='35',
        description='HSV色调最大值')

    hsv_s_min_arg = DeclareLaunchArgument(
        'hsv_s_min', default_value='80',
        description='HSV饱和度最小值')

    hsv_s_max_arg = DeclareLaunchArgument(
        'hsv_s_max', default_value='255',
        description='HSV饱和度最大值')

    hsv_v_min_arg = DeclareLaunchArgument(
        'hsv_v_min', default_value='80',
        description='HSV明度最小值')

    hsv_v_max_arg = DeclareLaunchArgument(
        'hsv_v_max', default_value='255',
        description='HSV明度最大值')

    # ============================================================
    # ROI感兴趣区域参数（默认取下半部分）
    # ============================================================
    roi_top_arg = DeclareLaunchArgument(
        'roi_top', default_value='240',
        description='ROI上边界（像素行号，推荐240~300）')

    roi_bottom_arg = DeclareLaunchArgument(
        'roi_bottom', default_value='480',
        description='ROI下边界（像素行号，通常为图像高度）')

    roi_left_arg = DeclareLaunchArgument(
        'roi_left', default_value='0',
        description='ROI左边界（像素列号）')

    roi_right_arg = DeclareLaunchArgument(
        'roi_right', default_value='640',
        description='ROI右边界（像素列号，通常为图像宽度）')

    # ============================================================
    # 轮廓/形态学参数
    # ============================================================
    min_contour_area_arg = DeclareLaunchArgument(
        'min_contour_area', default_value='200.0',
        description='最小轮廓面积（滤除噪声，推荐100~500）')

    morph_kernel_size_arg = DeclareLaunchArgument(
        'morph_kernel_size', default_value='5',
        description='形态学操作核大小（推荐3~7，奇数）')

    detection_method_arg = DeclareLaunchArgument(
        'detection_method', default_value='0',
        description='检测方法（0=轮廓加权质心, 1=滑动窗口）')

    # ============================================================
    # 话题参数
    # ============================================================
    sub_img_topic_arg = DeclareLaunchArgument(
        'sub_img_topic', default_value='/nv12_img',
        description='订阅的共享内存图像话题')

    pub_topic_arg = DeclareLaunchArgument(
        'pub_topic', default_value='/yellow_track_center',
        description='发布的车道线中心点话题')

    # ============================================================
    # 调试参数
    # ============================================================
    debug_output_arg = DeclareLaunchArgument(
        'debug_output', default_value='true',
        description='是否输出调试日志（true/false）')

    visualize_arg = DeclareLaunchArgument(
        'visualize', default_value='false',
        description='是否发布可视化图像到 /yellow_track_debug（true/false）')

    # ============================================================
    # 黄色车道线检测节点
    # ============================================================
    yellow_track_node = Node(
        package='yellow_track_opencv',
        executable='yellow_track_opencv',
        name='yellow_track_opencv',
        output='screen',
        parameters=[{
            # HSV参数
            'hsv_h_min': LaunchConfiguration('hsv_h_min'),
            'hsv_h_max': LaunchConfiguration('hsv_h_max'),
            'hsv_s_min': LaunchConfiguration('hsv_s_min'),
            'hsv_s_max': LaunchConfiguration('hsv_s_max'),
            'hsv_v_min': LaunchConfiguration('hsv_v_min'),
            'hsv_v_max': LaunchConfiguration('hsv_v_max'),
            # ROI参数
            'roi_top': LaunchConfiguration('roi_top'),
            'roi_bottom': LaunchConfiguration('roi_bottom'),
            'roi_left': LaunchConfiguration('roi_left'),
            'roi_right': LaunchConfiguration('roi_right'),
            # 轮廓处理参数
            'min_contour_area': LaunchConfiguration('min_contour_area'),
            'morph_kernel_size': LaunchConfiguration('morph_kernel_size'),
            'detection_method': LaunchConfiguration('detection_method'),
            # 话题参数
            'sub_img_topic': LaunchConfiguration('sub_img_topic'),
            'pub_topic': LaunchConfiguration('pub_topic'),
            # 调试参数
            'debug_output': LaunchConfiguration('debug_output'),
            'visualize': LaunchConfiguration('visualize'),
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    )

    return LaunchDescription([
        # 声明参数
        hsv_h_min_arg, hsv_h_max_arg,
        hsv_s_min_arg, hsv_s_max_arg,
        hsv_v_min_arg, hsv_v_max_arg,
        roi_top_arg, roi_bottom_arg,
        roi_left_arg, roi_right_arg,
        min_contour_area_arg, morph_kernel_size_arg,
        detection_method_arg,
        sub_img_topic_arg, pub_topic_arg,
        debug_output_arg, visualize_arg,
        # 节点
        yellow_track_node,
    ])
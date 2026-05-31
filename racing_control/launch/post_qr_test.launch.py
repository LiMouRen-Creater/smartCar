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

【图像链路】
  USB Camera
      ↓ shared_mem JPEG
  /hbmem_img
      ↓ hobot_codec_decode
  /nv12_img
      ↓ shared_mem NV12
  YOLO障碍物检测 / ResNet黑线检测 / OpenCV黄色线检测
      ↓
  检测结果话题
      ↓
  image_annotator 叠加检测结果
      ↓
  /image_annotated

【显示链路】
  /nv12_img
      ↓ hobot_codec_encode
  /img_decode

【启动方式】
  ros2 launch racing_control post_qr_test.launch.py

=====================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ================================================================
    # 0. 声明启动参数
    # ================================================================
    launch_args = [
        # 相机参数
        DeclareLaunchArgument(
            'device',
            default_value='/dev/video0',
            description='USB摄像头设备路径'
        ),

        # 后QR控制参数
        DeclareLaunchArgument(
            'post_qr_forward_speed',
            default_value='0.5',
            description='Phase1左转速度(m/s)'
        ),
        DeclareLaunchArgument(
            'post_qr_forward_time_ms',
            default_value='3000',
            description='Phase1左转时间(ms)，约1.5米'
        ),
        DeclareLaunchArgument(
            'post_qr_forward_angular_z',
            default_value='0.3',
            description='Phase1左转角速度(rad/s)'
        ),
        DeclareLaunchArgument(
            'post_qr_turn_speed',
            default_value='0.3',
            description='Phase2右转角速度(rad/s)'
        ),
        DeclareLaunchArgument(
            'post_qr_turn_timeout_ms',
            default_value='3000',
            description='Phase2右转超时(ms)'
        ),
        DeclareLaunchArgument(
            'post_qr_yellow_area_threshold',
            default_value='0.15',
            description='黄色面积占比阈值，超过该值认为进入黄色道路区域'
        ),
        DeclareLaunchArgument(
            'post_qr_follow_speed',
            default_value='0.4',
            description='Phase3黄色巡线速度(m/s)'
        ),
        DeclareLaunchArgument(
            'post_qr_follow_kp',
            default_value='0.005',
            description='Phase3黄色巡线比例系数'
        ),

        # 锥桶避障参数
        DeclareLaunchArgument(
            'avoid_x',
            default_value='0.8',
            description='锥桶避障速度(m/s)'
        ),
        DeclareLaunchArgument(
            'avoid_kp',
            default_value='0.0035',
            description='锥桶避障比例系数'
        ),
        DeclareLaunchArgument(
            'end_y',
            default_value='190',
            description='锥桶触发避障的底部y阈值'
        ),

        # OpenCV黄色车道线检测参数
        DeclareLaunchArgument('hsv_h_min', default_value='15'),
        DeclareLaunchArgument('hsv_h_max', default_value='35'),
        DeclareLaunchArgument('hsv_s_min', default_value='80'),
        DeclareLaunchArgument('hsv_s_max', default_value='255'),
        DeclareLaunchArgument('hsv_v_min', default_value='80'),
        DeclareLaunchArgument('hsv_v_max', default_value='255'),
        DeclareLaunchArgument('roi_top', default_value='240'),
        DeclareLaunchArgument('roi_bottom', default_value='480'),
        DeclareLaunchArgument('visualize', default_value='true'),
        DeclareLaunchArgument('debug_output', default_value='true'),
    ]

    # ================================================================
    # 1. USB相机节点
    #
    # 输入：USB摄像头 /dev/video0
    # 输出：/hbmem_img
    # 格式：JPEG
    # 模式：shared_mem，zero-copy开启
    # ================================================================
    usb_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_usb_cam'),
                'launch',
                'hobot_usb_cam.launch.py'
            )
        ),
        launch_arguments={
            'usb_image_width': '640',
            'usb_image_height': '480',
            'usb_zero_copy': 'True',
            'usb_video_device': LaunchConfiguration('device'),
        }.items(),
    )

    # ================================================================
    # 2. JPEG → NV12 解码节点
    #
    # 输入：/hbmem_img
    # 输出：/nv12_img
    # 输入格式：jpeg
    # 输出格式：nv12
    # 输入模式：shared_mem
    # 输出模式：shared_mem
    #
    # 后面的 YOLO / ResNet / 黄色检测都应该处理 /nv12_img
    # ================================================================
    nv12_decode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch',
                'hobot_codec_decode.launch.py'
            )
        ),
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
        }.items(),
    )

    # ================================================================
    # 3. NV12 → JPEG 编码节点
    #
    # 输入：/nv12_img
    # 输出：/img_decode
    # 作用：保留你原来能看到原始画面的显示链路
    #
    # 注意：
    # /img_decode 只是给可视化用；
    # YOLO / ResNet / 黄色检测仍然应该吃 /nv12_img。
    # ================================================================
    img_encode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch',
                'hobot_codec_encode.launch.py'
            )
        ),
        launch_arguments={
            'codec_channel': '2',
            'codec_jpg_quality': '80.0',
            'codec_in_format': 'nv12',
            'codec_out_format': 'jpeg',
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/nv12_img',
            'codec_pub_topic': '/img_decode',
        }.items(),
    )

    # # ================================================================
    # # 4. 车辆底盘节点
    # # ================================================================
    # origincar_base = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('origincar_base'),
    #             'launch',
    #             'base_serial.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'akmcar': 'false'
    #     }.items(),
    # )

    # ================================================================
    # 5. OpenCV黄色车道线检测节点
    #
    # 输入：/nv12_img
    # 输出：/yellow_track_center
    # 作用：后QR黄色道路巡线控制
    #
    # 这里不用 topic_tools，不再转发 /yellow_track_center。
    # 控制节点直接订阅 /yellow_track_center 即可。
    # ================================================================
    yellow_track_opencv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yellow_track_opencv'),
                'launch',
                'yellow_track_opencv.launch.py'
            )
        ),
        launch_arguments={
            'hsv_h_min': LaunchConfiguration('hsv_h_min'),
            'hsv_h_max': LaunchConfiguration('hsv_h_max'),
            'hsv_s_min': LaunchConfiguration('hsv_s_min'),
            'hsv_s_max': LaunchConfiguration('hsv_s_max'),
            'hsv_v_min': LaunchConfiguration('hsv_v_min'),
            'hsv_v_max': LaunchConfiguration('hsv_v_max'),
            'roi_top': LaunchConfiguration('roi_top'),
            'roi_bottom': LaunchConfiguration('roi_bottom'),

            # 关键：黄色检测订阅解码后的 NV12 图像
            'sub_img_topic': '/nv12_img',

            # 如果 yellow_track_opencv.launch.py 支持共享内存参数，会生效；
            # 如果该 launch 文件没有声明这些参数，启动时报 unused argument 时再删掉。
            'is_shared_mem_sub': '1',
            'image_format': 'nv12',

            'visualize': LaunchConfiguration('visualize'),
            'debug_output': LaunchConfiguration('debug_output'),
            'pub_topic': '/yellow_track_center',
        }.items(),
    )

    # # ================================================================
    # # 6. ResNet黑线检测节点
    # #
    # # 输入：/nv12_img
    # # 输出：通常为 /racing_track_center_detection
    # #
    # # 注意：
    # # 如果 racing_track_detection_resnet.launch.py 内部已经默认订阅 /nv12_img，
    # # 下面这样直接 Include 就可以。
    # # 如果它内部默认订阅 /hbmem_img，需要去它自己的 launch 文件里
    # # 把输入图像话题改成 /nv12_img，并开启 is_shared_mem_sub。
    # # ================================================================
    # racing_track_detection_resnet = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('racing_track_detection_resnet'),
    #             'launch',
    #             'racing_track_detection_resnet.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         # 常见参数名，若对应 launch 未声明，删除这几项即可
    #         'sub_img_topic': '/nv12_img',
    #         'image_topic': '/nv12_img',
    #         'is_shared_mem_sub': '1',
    #         'image_format': 'nv12',
    #     }.items(),
    # )

    # # ================================================================
    # # 7. YOLO锥桶障碍物检测节点
    # #
    # # 输入：/nv12_img
    # # 输出：YOLO障碍物检测结果话题
    # #
    # # 注意：
    # # 如果 racing_obstacle_detection_yolo.launch.py 内部默认订阅 /hbmem_img，
    # # 必须进入该 launch 文件或节点参数文件，把输入图像改为 /nv12_img。
    # # ================================================================
    # racing_obstacle_detection_yolo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('racing_obstacle_detection_yolo'),
    #             'launch',
    #             'racing_obstacle_detection_yolo.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         # 常见参数名，若对应 launch 未声明，删除这几项即可
    #         'sub_img_topic': '/nv12_img',
    #         'image_topic': '/nv12_img',
    #         'is_shared_mem_sub': '1',
    #         'image_format': 'nv12',
    #         'input_image_width': '640',
    #         'input_image_height': '480',
    #     }.items(),
    # )

    # # ================================================================
    # # 8. QR码检测节点
    # #
    # # 触发后QR状态机
    # # 如果 qrcode 节点内部需要图像，也建议让它保持原来能跑的默认写法。
    # # ================================================================
    # qrcode_node = Node(
    #     package='qrcode',
    #     executable='qrcode',
    #     name='qrcode',
    #     output='screen',
    #     arguments=['--ros-args', '--log-level', 'info'],
    # )

    # ================================================================
    # 9. 图像标注节点
    #
    # 输入：
    #   /img_decode                         原图显示流
    #   /yellow_track_center                黄色车道线中心点
    #   /racing_track_center_detection      ResNet黑线检测结果
    #   YOLO障碍物检测结果话题
    #
    # 输出：
    #   /image_annotated
    #
    # 说明：
    # 这里用 /img_decode 做底图，是因为它是普通 ROS 图像，方便显示；
    # 算法本身仍然处理 /nv12_img，不破坏 shared_mem 算法链路。
    #
    # 如果你的 image_annotator 源码参数名不同，需要按源码中的 declare_parameter 名称调整。
    # ==========================
    image_annotator = Node(
        package='image_annotator',
        executable='image_annotator',
        name='image_annotator',
        output='screen',
        parameters=[{
            # 原图显示流
            'raw_image_topic': '/img_decode',
            'image_topic': '/img_decode',
            'sub_img_topic': '/img_decode',

            # 黄色车道线检测结果
            'yellow_center_topic': '/yellow_track_center',

            # ResNet黑线检测结果
            'track_result_topic': '/racing_track_center_detection',
            'resnet_result_topic': '/racing_track_center_detection',

            # YOLO检测结果话题
            # 如果你的YOLO实际输出话题不是这个，运行 ros2 topic list 后改这里
            'yolo_result_topic': '/racing_obstacle_detection',
            'obstacle_result_topic': '/racing_obstacle_detection',

            # 输出标注图
            'output_topic': '/image_annotated',
            'pub_topic': '/image_annotated',

            # 输出缩放
            'resize_scale': 0.5,
            'output_width': 320,
            'output_height': 240,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    )

    # ================================================================
    # 10. 后QR码控制节点
    #
    # 输入：
    #   QR码触发信号
    #   /yellow_track_center
    #   /racing_track_center_detection
    #   YOLO障碍物检测结果
    #
    # 输出：
    #   /cmd_vel
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
    # 11. rosbridge WebSocket
    #
    # 用于网页端查看 /img_decode 或 /image_annotated。
    # ================================================================
    rosbridge_node = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'rosbridge_server',
            'rosbridge_websocket_launch.xml'
        ],
        output='screen'
    )

    # ================================================================
    # 组装启动顺序
    # ================================================================
    return LaunchDescription(launch_args + [
        # 1. 图像底层链路
        usb_cam,                       # /hbmem_img
        nv12_decode,                   # /hbmem_img → /nv12_img
        img_encode,                    # /nv12_img → /img_decode

        # # 2. 底盘
        # origincar_base,

        # # 3. 感知算法，全部基于 /nv12_img
        # racing_track_detection_resnet,
        # racing_obstacle_detection_yolo,
        yellow_track_opencv,
        # qrcode_node,

        # # 4. 可视化叠加
        image_annotator,

        # # 5. 后QR控制
        # post_qr_control_node,

        # 6. 网页显示
        rosbridge_node,
    ])
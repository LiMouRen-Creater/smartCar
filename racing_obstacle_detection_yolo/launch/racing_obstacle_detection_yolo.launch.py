import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    image_width_launch_arg = DeclareLaunchArgument(
        "dnn_sample_image_width", default_value=TextSubstitution(text="640")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "dnn_sample_image_height", default_value=TextSubstitution(text="480")
    )

    web_show = os.getenv('WEB_SHOW')
    print("web_show is ", web_show)


    # jpeg图片编码&发布pkg（务必加上 codec_channel: '2'）
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_channel': '2',              # 💡 极其重要：必须指定为通道2，否则会和解码器冲突导致崩溃！
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_in_format': 'nv12',         
            'codec_out_format': 'jpeg',        
            'codec_sub_topic': '/nv12_img',    
            'codec_pub_topic': '/image_jpeg',  
        }.items()
    )

    # 网页传输节点（补全图像类型参数 mjpeg）
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg',
            'websocket_image_type': 'mjpeg',   # 💡 补上这个关键参数！
            'websocket_smart_topic': '/racing_obstacle_detection'
        }.items()
    )

    # 障碍物检测pkg
    racing_obstacle_detection_yolov5_node = Node(
        package='racing_obstacle_detection_yolo',
        executable='racing_obstacle_detection_yolo',
        output='screen',
        parameters=[
            {"is_shared_mem_sub": True},
            {"sub_img_topic": "/nv12_img"},
            {"config_file": "/root/dev_ws/src/racing_obstacle_detection_yolo/config/yolov5sconfig.json"},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    if web_show == "TRUE":
        return LaunchDescription([
            racing_obstacle_detection_yolov5_node,
            jpeg_codec_node,
            web_node
        ])
    else:
        return LaunchDescription([
            racing_obstacle_detection_yolov5_node
        ])
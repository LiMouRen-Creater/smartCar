import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Copy your YOLO model config files (replace default dnn_node_example config)
    racing_obstacle_config_dir = os.path.join(
        get_package_prefix('racing_obstacle_detection_yolo'),
        "lib/racing_obstacle_detection_yolo/config"
    )
    os.system(f"cp {racing_obstacle_config_dir}/yolov5sconfig.json ./config/")
    os.system(f"cp {racing_obstacle_config_dir}/obstacles.list ./config/")
    os.system(f"cp {racing_obstacle_config_dir}/converted_model.bin ./config/")

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument("dnn_example_config_file", default_value=TextSubstitution(text="config/yolov5sconfig.json")),
        DeclareLaunchArgument("dnn_example_dump_render_img", default_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("dnn_example_image_width", default_value=TextSubstitution(text="480")),
        DeclareLaunchArgument("dnn_example_image_height", default_value=TextSubstitution(text="272")),
        DeclareLaunchArgument("dnn_example_msg_pub_topic_name", default_value=TextSubstitution(text="hobot_dnn_detection")),
        DeclareLaunchArgument('device', default_value='/dev/video0', description='usb camera device'),
    ]

    # Include launch descriptions
    usb_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'),
                                       launch_arguments={'usb_image_width': '640', 'usb_image_height': '480',
                                                         'usb_video_device': LaunchConfiguration('device')}.items())

    nv12_codec_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'ros', 'codec_out_mode': 'shared_mem',
                                                                 'codec_sub_topic': '/image', 'codec_pub_topic': '/hbmem_img'}.items())

    jpeg_codec_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'shared_mem', 'codec_out_mode': 'ros',
                                                                 'codec_sub_topic': '/hbmem_img', 'codec_pub_topic': '/image'}.items())

    web_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
                                        launch_arguments={'websocket_image_topic': '/image', 'websocket_image_type': 'mjpeg',
                                                          'websocket_smart_topic': LaunchConfiguration("dnn_example_msg_pub_topic_name")}.items())

    # Algorithm node (now using your YOLO model!)
    dnn_node_example_node = Node(
        package='dnn_node_example',
        executable='example',
        output='screen',
        parameters=[
            {"config_file": LaunchConfiguration('dnn_example_config_file')},
            {"dump_render_img": LaunchConfiguration('dnn_example_dump_render_img')},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {"msg_pub_topic_name": LaunchConfiguration("dnn_example_msg_pub_topic_name")}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    image_transport_node = Node(
        package='utils',
        executable='image_transport_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    ##上面是websocket的节点所有节点，共五个，全部开启了之后才可以在网页端看到图像和检测结果
    ##下面是小车启动之后的节点

    #1. 启动rosbridge_server节点，开启8000端口的websocket服务
    rosbridge_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )

    #2. 启动视觉语言模型节点
    vision_language_model = Node(
        package='vision_language_model',
        executable='vision_language_model',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    #3. 启动图像转模型节点
    img_to_model = Node(
        package='img_to_model',
        executable='img_to_model',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    #4. 启动二维码识别节点
    qrcode = Node(
        package='qrcode',
        executable='qrcode',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    racing_obstacle_detection_yolo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_obstacle_detection_yolo') + '/launch/racing_obstacle_detection_yolo.launch.py'))

    racing_track_detection_resnet = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_track_detection_resnet') + '/launch/racing_track_detection_resnet.launch.py'))
    
    origincar_base = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('origincar_base') + '/launch/origincar_bringup.launch.py'))
    
    racing_control = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_control') + '/launch/racing_control.launch.py'))





    return LaunchDescription(launch_args + [
        usb_node,
        nv12_codec_node,
        dnn_node_example_node,
        web_node,
        image_transport_node,
        


        rosbridge_node
        






    ])
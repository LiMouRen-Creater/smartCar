import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Copy config files
    dnn_node_example_path = os.path.join(get_package_prefix('dnn_node_example'), "lib/dnn_node_example")
    os.system(f"cp -r {dnn_node_example_path}/config .")

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument("dnn_example_config_file", default_value=TextSubstitution(text="config/fcosworkconfig.json")),
        DeclareLaunchArgument("dnn_example_dump_render_img", default_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("dnn_example_image_width", default_value=TextSubstitution(text="480")),
        DeclareLaunchArgument("dnn_example_image_height", default_value=TextSubstitution(text="272")),
        DeclareLaunchArgument("dnn_example_msg_pub_topic_name", default_value=TextSubstitution(text="hobot_dnn_detection")),
        DeclareLaunchArgument('device', default_value='/dev/video0', description='usb camera device'),
    ]

    rosbridge_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )

    # Include launch descriptions
    usb_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'),
                                       launch_arguments={'usb_image_width': '640', 'usb_image_height': '480','usb_zero_copy': 'True',
                                                         'usb_video_device': LaunchConfiguration('device')}.items())

    nv12_decode_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
                                               launch_arguments={'codec_channel'  : '1',
                                                                 'codec_in_format':'jpeg',        'codec_out_format': 'nv12',
                                                                 'codec_in_mode'  : 'shared_mem', 'codec_out_mode'  : 'shared_mem',
                                                                 'codec_sub_topic': '/hbmem_img', 'codec_pub_topic' : '/nv12_img'}.items())

    img_encode_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'),
                                               launch_arguments={'codec_channel'  : '2',             'codec_jpg_quality': '70.0',  'codec_output_framerate' : '30',
                                                                 'codec_in_format': 'nv12',          'codec_out_format' : 'jpeg',
                                                                 'codec_in_mode'  : 'shared_mem',    'codec_out_mode'   : 'ros',
                                                                 'codec_sub_topic': '/nv12_img', 'codec_pub_topic'  : '/jpeg_img'}.items())
                                                                 
    web_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
                                        launch_arguments={'websocket_image_topic': '/jpeg_img', 'websocket_image_type': 'mjpeg', # racing_obstacle_detection
                                                          'websocket_smart_topic': '/racing_obstacle_detection'}.items())    # racing_track_center_detection

    racing_track_detection_resnet_go = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_track_detection_resnet_go') + '/launch/racing_track_detection_resnet.launch.py'))

    racing_track_detection_resnet_s = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_track_detection_resnet_s') + '/launch/racing_track_detection_resnet.launch.py'))

    racing_track_detection_resnet_n = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_track_detection_resnet_n') + '/launch/racing_track_detection_resnet.launch.py'))

    racing_track_detection_resnet_back = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_track_detection_resnet_back') + '/launch/racing_track_detection_resnet.launch.py'))


    racing_obstacle_detection_yolo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_obstacle_detection_yolo') + '/launch/racing_obstacle_detection_yolo.launch.py'))

    origincar_base = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('origincar_base') + '/launch/origincar_bringup.launch.py'))
    
    racing_control = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_control') + '/launch/racing_control.launch.py'))
    # Algorithm node
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

    vision_language_model = Node(
        package='vision_language_model',
        executable='vision_language_model',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    qrcode = Node(
        package='qrcode',
        executable='qrcode',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    qrcode_tts_announce = Node(
        package='qrcode_tts_announce',
        executable='qrcode_tts_announce',
        name='qrcode_tts_announce',
        output='screen',
        parameters=[{
            'qrcode_topic': '/qrcode_result',
            'tts_topic': '/tts_text',
            'announce_once': True,
        }]
    )


    hobot_tts = Node(
        package='hobot_tts',
        executable='hobot_tts',
        name='hobot_tts',
        output='screen',
        parameters=[{
            # TTS 订阅文本话题。
            'topic_sub': '/tts_text',
            # 当前设备使用 USB 声卡；功能包默认值仍为 hw:0,1。
            'playback_device': 'plughw:2,0',
            # PCM 播放音量倍数。
            'volume_gain': 1.0,
            # 预热“二维码”、数字和方向词。首次生成后会写入磁盘，
            # 后续启动直接从 pcm_cache_dir 读取并装入内存。
            'warmup_enabled': True,
            # 保存二维码短片段、普通文本和可选常用字 PCM。
            'disk_cache_enabled': True,
            # PCM 磁盘缓存默认保存在源码功能包 hobot_tts-develop/pcm_cache。
            # 开启后会后台生成 3500 常用字 PCM，并在下次启动时复用。
            # 普通整句播报目前不会自动拆字拼接。
            # 该任务较重，曾在当前设备日志中伴随 TTS 段错误，因此默认关闭。
            'common_chars_cache_enabled': False,
            # 3500 常用字列表，使用功能包安装目录动态解析路径。
            'common_chars_file': get_package_share_directory('hobot_tts') +
                                 '/resources/common_3500_chars.txt',
        }]
    )

    return LaunchDescription(launch_args + [
        rosbridge_node,
        usb_node,
        nv12_decode_node,
        img_encode_node,  
        qrcode,
        racing_track_detection_resnet_go,
        racing_track_detection_resnet_s,
        racing_track_detection_resnet_n,
        racing_track_detection_resnet_back,
        racing_obstacle_detection_yolo,
        origincar_base,
        web_node,
        qrcode_tts_announce,
        hobot_tts,
        # vision_language_model,
        # racing_control,
    ])

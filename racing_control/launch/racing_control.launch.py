import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # ======== 黑线巡线参数 ========
        # 0.8    0.006
        # 1.5    0.01
        DeclareLaunchArgument('line_x',default_value='0.8'),
        DeclareLaunchArgument('line_kp',default_value='0.006'),
        
        # ======== 锥桶避障参数 ========
        DeclareLaunchArgument('avoid_x',default_value='0.8'),
        DeclareLaunchArgument('avoid_kp',default_value='0.0035'),
        
        # ======== 黑线检测参数 ========
        DeclareLaunchArgument('end_y',default_value='190'),      # 4根黑线
        DeclareLaunchArgument('y_stop_p',default_value='460'),
        DeclareLaunchArgument('y_avoid_dir_p',default_value='200'),
        DeclareLaunchArgument('y_dir_qrcode',default_value='155'),        # 14根黑线
        DeclareLaunchArgument('y_avoid_dir_qrcode',default_value='170'),  # 7根黑线
        
        # ======== 【后QR码状态机参数】========
        # Phase 1: 开环直行（QR码扫描完后向前走到通道口）
        DeclareLaunchArgument('post_qr_forward_speed', default_value='0.5',
                              description='直行速度(m/s)'),
        DeclareLaunchArgument('post_qr_forward_time_ms', default_value='3000',
                              description='直行持续时间(ms)'),
        
        # Phase 2: 右转对准黄色道路
        DeclareLaunchArgument('post_qr_turn_speed', default_value='0.3',
                              description='右转角速度(rad/s，负值为右)'),
        DeclareLaunchArgument('post_qr_turn_timeout_ms', default_value='3000',
                              description='右转最大超时(ms)'),
        
        # 黄色检测判定
        DeclareLaunchArgument('post_qr_yellow_area_threshold', default_value='0.15',
                              description='黄色面积占比阈值≥此值认为进入通道'),
        
        # Phase 3: 黄色车道线巡线
        DeclareLaunchArgument('post_qr_follow_speed', default_value='0.4',
                              description='黄色巡线速度(m/s)'),
        DeclareLaunchArgument('post_qr_follow_kp', default_value='0.005',
                              description='黄色巡线比例系数'),
        
        Node(
            package='racing_control',
            executable='racing_control',
            output='screen',
            parameters=[
                # 黑线/避障参数
                {"pub_control_topic": '/racing'},
                {"line_x": LaunchConfiguration('line_x')},
                {"line_kp": LaunchConfiguration('line_kp')},
                {"avoid_x": LaunchConfiguration('avoid_x')},
                {"avoid_kp": LaunchConfiguration('avoid_kp')},
                {"end_y": LaunchConfiguration('end_y')},
                {"y_stop_p": LaunchConfiguration('y_stop_p')},
                {"y_avoid_dir_p": LaunchConfiguration('y_avoid_dir_p')},
                {"y_dir_qrcode": LaunchConfiguration('y_dir_qrcode')},
                {"y_avoid_dir_qrcode": LaunchConfiguration('y_avoid_dir_qrcode')},
                
                # 后QR参数
                {"post_qr_forward_speed": LaunchConfiguration('post_qr_forward_speed')},
                {"post_qr_forward_time_ms": LaunchConfiguration('post_qr_forward_time_ms')},
                {"post_qr_turn_speed": LaunchConfiguration('post_qr_turn_speed')},
                {"post_qr_turn_timeout_ms": LaunchConfiguration('post_qr_turn_timeout_ms')},
                {"post_qr_yellow_area_threshold": LaunchConfiguration('post_qr_yellow_area_threshold')},
                {"post_qr_follow_speed": LaunchConfiguration('post_qr_follow_speed')},
                {"post_qr_follow_kp": LaunchConfiguration('post_qr_follow_kp')},
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='racing_control',
            executable='control_master',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])
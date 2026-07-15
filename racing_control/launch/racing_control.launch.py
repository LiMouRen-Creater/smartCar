import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    return LaunchDescription([
        # 0.8    0.006
        # 1.5    0.01
        DeclareLaunchArgument('line_x',default_value='0.8'),
        DeclareLaunchArgument('line_kp',default_value='0.006'),
        
        DeclareLaunchArgument('avoid_x',default_value='0.8'),
        DeclareLaunchArgument('avoid_kp',default_value='0.0035'),
        
        DeclareLaunchArgument('end_y',default_value='190'),      # 4根黑线

        DeclareLaunchArgument('y_stop_p',default_value='460'),
        DeclareLaunchArgument('y_avoid_dir_p',default_value='200'),

        DeclareLaunchArgument('y_dir_qrcode',default_value='155'),        # 14根黑线
        DeclareLaunchArgument('y_avoid_dir_qrcode',default_value='170'),  # 7根黑线
        
        Node(
            package='racing_control',
            executable='racing_control',
            output='screen',
            parameters=[
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
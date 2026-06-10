# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 巡黑线
        DeclareLaunchArgument('line_x',   default_value='0.8'),
        DeclareLaunchArgument('line_kp',  default_value='0.006'),
        # 避障
        DeclareLaunchArgument('avoid_x',  default_value='0.8'),
        DeclareLaunchArgument('avoid_kp', default_value='0.0035'),
        # 锥桶触发bottom阈值
        DeclareLaunchArgument('end_y',    default_value='190'),
        # P点停车bottom阈值
        DeclareLaunchArgument('y_stop_p', default_value='460'),
        # 二维码方向判断
        DeclareLaunchArgument('y_dir_qrcode',       default_value='155'),
        DeclareLaunchArgument('y_avoid_dir_qrcode', default_value='170'),
        # 追踪"通"字阈值（入口/出口）
        DeclareLaunchArgument('channel_enter_y', default_value='400'),
        DeclareLaunchArgument('channel_end_y',   default_value='300'),
        # REVERSE_TURN
        DeclareLaunchArgument('reverse_turn_angle',  default_value='1.57'),
        DeclareLaunchArgument('reverse_linear_x',    default_value='-0.3'),
        DeclareLaunchArgument('reverse_angular_z',   default_value='0.5'),
        # YELLOW_ENTER
        DeclareLaunchArgument('yellow_enter_angular_z', default_value='0.5'),
        # YELLOW_FOLLOW
        DeclareLaunchArgument('yellow_x',  default_value='0.5'),
        DeclareLaunchArgument('yellow_kp', default_value='0.005'),
        # EXIT_CHANNEL
        DeclareLaunchArgument('exit_linear_x',  default_value='0.5'),
        DeclareLaunchArgument('exit_angular_z', default_value='-0.3'),
        # P_TRACK
        DeclareLaunchArgument('p_track_x',  default_value='0.5'),
        DeclareLaunchArgument('p_track_kp', default_value='0.004'),

        Node(
            package='racing_control',
            executable='racing_control_fsm',
            output='screen',
            parameters=[{
                'line_x':   LaunchConfiguration('line_x'),
                'line_kp':  LaunchConfiguration('line_kp'),
                'avoid_x':  LaunchConfiguration('avoid_x'),
                'avoid_kp': LaunchConfiguration('avoid_kp'),
                'end_y':    LaunchConfiguration('end_y'),
                'y_stop_p': LaunchConfiguration('y_stop_p'),
                'y_dir_qrcode':       LaunchConfiguration('y_dir_qrcode'),
                'y_avoid_dir_qrcode': LaunchConfiguration('y_avoid_dir_qrcode'),
                'channel_enter_y': LaunchConfiguration('channel_enter_y'),
                'channel_end_y':   LaunchConfiguration('channel_end_y'),
                'reverse_turn_angle': LaunchConfiguration('reverse_turn_angle'),
                'reverse_linear_x':   LaunchConfiguration('reverse_linear_x'),
                'reverse_angular_z':  LaunchConfiguration('reverse_angular_z'),
                'yellow_enter_angular_z': LaunchConfiguration('yellow_enter_angular_z'),
                'yellow_x':  LaunchConfiguration('yellow_x'),
                'yellow_kp': LaunchConfiguration('yellow_kp'),
                'exit_linear_x':  LaunchConfiguration('exit_linear_x'),
                'exit_angular_z': LaunchConfiguration('exit_angular_z'),
                'p_track_x':  LaunchConfiguration('p_track_x'),
                'p_track_kp': LaunchConfiguration('p_track_kp'),
            }],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        # control_master保持不变
        Node(
            package='racing_control',
            executable='control_master',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('v_line', default_value='1.0'),
        DeclareLaunchArgument('kp_line', default_value='0.006'),
        DeclareLaunchArgument('y_line', default_value='240'),

        DeclareLaunchArgument('v_avoid', default_value='0.8'),
        DeclareLaunchArgument('kp_avoid', default_value='0.0035'),

        DeclareLaunchArgument('y_zt', default_value='155'),
        DeclareLaunchArgument('y_p', default_value='435'), 
        DeclareLaunchArgument('y_qrcode', default_value='167'),
        
        Node(
            package='control',
            executable='control_yolo',
            name='control_yolo',
            output='screen',
            parameters=[{
                'v_avoid': LaunchConfiguration('v_avoid'),
                'kp_avoid': LaunchConfiguration('kp_avoid'),

                'y_p': LaunchConfiguration('y_p'),
                'y_zt': LaunchConfiguration('y_zt'),
                'y_qrcode': LaunchConfiguration('y_qrcode'),
            }]
        ),
        
        Node(
            package='control',
            executable='control_resnet',
            name='control_resnet',
            output='screen',
            parameters=[{
                'kp_line': LaunchConfiguration('kp_line'),
                'v_line': LaunchConfiguration('v_line'),
                'y_line': LaunchConfiguration('y_line'),
            }]
        ),

        Node(
            package='control',
            executable='control_master',
            name='control_master',
            output='screen',
        ),
    ])

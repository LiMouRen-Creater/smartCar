# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

    web_show = os.getenv('WEB_SHOW')
    print("web_show is ", web_show)

    racing_track_detection_resnet18_node = Node(
        package='racing_track_detection_resnet',
        executable='racing_track_detection_resnet',
        output='screen',
        parameters=[
            {"sub_img_topic": "/nv12_img"},
            {"model_path": "/root/dev_ws/src/racing_track_detection_resnet/config/converted_model.bin"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 编码器和websocket由主launch统一管理，避免冲突
    return LaunchDescription([
        racing_track_detection_resnet18_node
    ])
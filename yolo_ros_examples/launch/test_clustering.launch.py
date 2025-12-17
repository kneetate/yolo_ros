# Copyright (C) 2024 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # モデルファイルの絶対パスを取得（srcディレクトリ直下のyolo_weightsを参照）
    model_path = os.path.expanduser(
        "~/fanuc_ros2_ws/src/pruning_ee_camera/yolo_weights/best0809_1919.pt"
    )

    yolo_server_node = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("yolo_bringup"),
                        "launch",
                        "yolo.launch.py",
                    )
                ),
                launch_arguments={
                    "as_server": LaunchConfiguration("as_server", default="True"),
                    "model": LaunchConfiguration("model", default=model_path),
                    "use_tracking": LaunchConfiguration("use_tracking", default="False"),
                    "device": LaunchConfiguration("device", default="cuda:0"),
                    "enable": LaunchConfiguration("enable", default="True"),
                    "threshold": LaunchConfiguration("threshold", default="0.25"),
                    "iou": LaunchConfiguration("iou", default="0.45"),
                    "input_image_topic": LaunchConfiguration(
                        "input_image_topic", default="/Scepter/color/image_raw"
                    ),
                    "image_reliability": LaunchConfiguration(
                        "image_reliability", default="1"
                    ),
                    "namespace": LaunchConfiguration("namespace", default="yolo_branch"),
                }.items(),
            )
        ]
    )

    example_client_node = Node(
        package="yolo_ros_examples",
        executable="yolo_seg_clustering",
        output="screen",
        namespace="yolo_branch",
    )

    ld = LaunchDescription()
    ld.add_action(yolo_server_node)
    ld.add_action(example_client_node)
    return ld

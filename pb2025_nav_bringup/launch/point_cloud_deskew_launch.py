# Copyright 2026 Jieliang Li
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    deskew_dir = get_package_share_directory("point_cloud_deskew")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    livox_params_file = LaunchConfiguration("livox_params_file")
    launch_livox = LaunchConfiguration("launch_livox")
    launch_robot_state_publisher = LaunchConfiguration("launch_robot_state_publisher")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    configured_livox_params = ParameterFile(
        RewrittenYaml(
            source_file=livox_params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_livox_ros_driver2_node = Node(
        condition=IfCondition(launch_livox),
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_livox_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_point_cloud_deskew_node = Node(
        package="point_cloud_deskew",
        executable="point_cloud_deskew_node",
        name="point_cloud_deskew",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    bringup_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            start_livox_ros_driver2_node,
            start_point_cloud_deskew_node,
        ]
    )

    start_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "robot_state_publisher_launch.py")
        ),
        condition=IfCondition(launch_robot_state_publisher),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "use_rviz": "False",
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Top-level namespace",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    deskew_dir, "config", "point_cloud_deskew.yaml"
                ),
                description="Full path to the point cloud deskew parameter file",
            ),
            DeclareLaunchArgument(
                "livox_params_file",
                default_value=os.path.join(
                    bringup_dir, "config", "reality", "nav2_params_mppi.yaml"
                ),
                description="Full path to the ROS 2 parameter file containing livox_ros_driver2",
            ),
            DeclareLaunchArgument(
                "launch_livox",
                default_value="True",
                description="Whether to launch livox_ros_driver2 together with point cloud deskew",
            ),
            DeclareLaunchArgument(
                "launch_robot_state_publisher",
                default_value="True",
                description="Whether to launch robot_state_publisher for TF required by point cloud deskew",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="False",
                description="Whether to respawn the point cloud deskew node if it crashes",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            start_robot_state_publisher,
            bringup_group,
        ]
    )

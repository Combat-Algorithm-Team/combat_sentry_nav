# Copyright 2025 Lihan Chen
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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _get_point_lio_additional_env():
    for libusb_path in [
        "/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0",
        "/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0",
    ]:
        if os.path.exists(libusb_path):
            return {"LD_PRELOAD": libusb_path}
    return {}


def _load_use_pcfilter(params_file_path):
    with open(params_file_path, "r", encoding="utf-8") as params_file:
        params = yaml.safe_load(params_file) or {}

    use_pcfilter = (
        params.get("pcfilter_bringup", {})
        .get("ros__parameters", {})
        .get("use_pcfilter", True)
    )

    if isinstance(use_pcfilter, str):
        return use_pcfilter.lower() in ["1", "true", "yes", "on"]

    return bool(use_pcfilter)


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    param_substitutions = {"use_sim_time": use_sim_time}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    use_pcfilter = _load_use_pcfilter(params_file.perform(context))
    point_lio_lid_topic = "livox/lidar_filtered" if use_pcfilter else "livox/lidar"
    point_lio_additional_env = _get_point_lio_additional_env()

    actions = [
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_ros_driver2",
            namespace=namespace,
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="point_lio",
            executable="pointlio_mapping",
            name="point_lio",
            namespace=namespace,
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=point_lio_additional_env,
            parameters=[configured_params, {"common.lid_topic": point_lio_lid_topic}],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]

    if use_pcfilter:
        actions.insert(
            1,
            Node(
                package="body_point_cloud_filter",
                executable="body_point_cloud_filter_node",
                name="body_point_cloud_filter",
                namespace=namespace,
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        )

    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "reality", "livox_pointlio_pcfilter_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(OpaqueFunction(function=_launch_setup))

    return ld

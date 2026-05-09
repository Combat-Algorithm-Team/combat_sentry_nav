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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    zones_file = LaunchConfiguration("zones_file")
    autostart = LaunchConfiguration("autostart")
    use_zone_monitor = LaunchConfiguration("use_zone_monitor")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "reality", "semantic_terrain_layer_test_params.yaml"
        ),
        description="Standalone semantic terrain layer test parameter file",
    )

    declare_zones_file_cmd = DeclareLaunchArgument(
        "zones_file",
        default_value=os.path.join(
            bringup_dir, "config", "reality", "semantic_terrain_layer_test_zones.yaml"
        ),
        description="Semantic terrain zone YAML to load into the standalone test costmap",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically configure and activate the standalone costmap",
    )

    declare_use_zone_monitor_cmd = DeclareLaunchArgument(
        "use_zone_monitor",
        default_value="True",
        description="Start terrain_zone_monitor for markers and state-topic checks",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level",
    )

    static_tf_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="semantic_terrain_test_static_tf",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "base_yaw_odom",
        ],
    )

    standalone_costmap_cmd = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="terrain_semantic_costmap_lifecycle_manager",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    terrain_zone_monitor_cmd = Node(
        condition=IfCondition(use_zone_monitor),
        package="pb_nav2_plugins",
        executable="terrain_zone_monitor",
        name="terrain_zone_monitor",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_zones_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_zone_monitor_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(static_tf_cmd)
    ld.add_action(standalone_costmap_cmd)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(terrain_zone_monitor_cmd)
    return ld

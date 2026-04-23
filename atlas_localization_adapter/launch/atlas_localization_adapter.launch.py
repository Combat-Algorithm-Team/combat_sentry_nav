# Copyright 2026 Lihan Chen
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    atlas_params_file = LaunchConfiguration("atlas_params_file")

    atlas_dir = get_package_share_directory("atlas_localization_adapter")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the node"
    )

    declare_atlas_params_file = DeclareLaunchArgument(
        "atlas_params_file",
        default_value=PathJoinSubstitution(
            [atlas_dir, "config", "reality", "atlas_localization_adapter.yaml"]
        ),
        description="Path to the Atlas localization adapter config file",
    )

    start_atlas_localization_adapter = Node(
        package="atlas_localization_adapter",
        executable="atlas_localization_adapter_node",
        name="atlas_localization_adapter",
        namespace=namespace,
        output="screen",
        remappings=remappings,
        parameters=[atlas_params_file],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_atlas_params_file)
    ld.add_action(start_atlas_localization_adapter)

    return ld

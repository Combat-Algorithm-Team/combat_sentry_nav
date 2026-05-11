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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def get_libusb_env():
    for libusb_path in [
        "/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0",
        "/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0",
    ]:
        if os.path.exists(libusb_path):
            return {"LD_PRELOAD": libusb_path}
    return {}


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    odin_dir = get_package_share_directory("odin_ros_driver")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    livox_params_file = LaunchConfiguration("livox_params_file")
    odin_config_file = LaunchConfiguration("odin_config_file")
    launch_odin = LaunchConfiguration("launch_odin")
    launch_livox = LaunchConfiguration("launch_livox")
    use_point_cloud_fusion = LaunchConfiguration("use_point_cloud_fusion")
    launch_robot_state_publisher = LaunchConfiguration("launch_robot_state_publisher")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
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

    start_odin_node = Node(
        condition=IfCondition(launch_odin),
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        additional_env=get_libusb_env(),
        parameters=[{"config_file": odin_config_file}],
        arguments=["--ros-args", "--log-level", log_level],
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
        package="sentry_fusion",
        executable="sentry_fusion_node",
        name="point_cloud_deskew",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"enable_fusion": ParameterValue(use_point_cloud_fusion, value_type=bool)},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Keep this node's parameters explicit so the current rclcpp executable does
    # not receive a duplicated use_sim_time override through the shared YAML.
    start_odom_adapter_node = Node(
        package="sentry_fusion",
        executable="odom_adapter_node",
        name="odom_adapter",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            {
                "input_odometry_topic": "odin1/odometry_highfreq",
                "lidar_odometry_topic": "lidar_odometry",
                "robot_base_odometry_topic": "odometry",
                "base_yaw_joint_topic": "base_yaw_joint_publisher",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "robot_base_frame": "base_yaw",
                "robot_base_odom_frame": "base_yaw_odom",
                "lidar_frame": "odin1_base_link",
                "tf_lookup_timeout_sec": 0.05,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_terrain_analysis_node = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_terrain_analysis_ext_node = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("cloud_in", "terrain_map_ext"),
            ("scan", "obstacle_scan"),
        ],
    )

    start_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/map_updates", "map_updates"),
        ],
    )

    start_map_saver_server_node = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["map_saver"]},
        ],
    )

    start_static_map_to_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    slam_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            start_odin_node,
            start_livox_ros_driver2_node,
            start_point_cloud_deskew_node,
            start_odom_adapter_node,
            start_terrain_analysis_node,
            start_terrain_analysis_ext_node,
            start_pointcloud_to_laserscan_node,
            start_slam_toolbox_node,
            start_map_saver_server_node,
            start_lifecycle_manager_node,
            start_static_map_to_odom_tf_node,
        ]
    )

    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(launch_rviz),
        launch_arguments={
            "namespace": namespace,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Top-level namespace",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    bringup_dir, "config", "reality", "nav2_params_mppi.yaml"
                ),
                description="Full path to the ROS 2 parameter file for SLAM nodes",
            ),
            DeclareLaunchArgument(
                "livox_params_file",
                default_value=os.path.join(
                    bringup_dir, "config", "reality", "nav2_params_mppi.yaml"
                ),
                description=(
                    "Full path to the ROS 2 parameter file containing livox_ros_driver2"
                ),
            ),
            DeclareLaunchArgument(
                "odin_config_file",
                default_value=os.path.join(odin_dir, "config", "control_command.yaml"),
                description="Full path to the Odin driver control config YAML",
            ),
            DeclareLaunchArgument(
                "launch_odin",
                default_value="True",
                description="Whether to launch the Odin1 host SDK driver",
            ),
            DeclareLaunchArgument(
                "launch_livox",
                default_value="True",
                description="Whether to launch livox_ros_driver2 for Livox/Odin fusion",
            ),
            DeclareLaunchArgument(
                "use_point_cloud_fusion",
                default_value="True",
                description="Fuse the deskewed Livox cloud with the Odin cloud",
            ),
            DeclareLaunchArgument(
                "launch_robot_state_publisher",
                default_value="True",
                description=(
                    "Whether to launch robot_state_publisher for required sensor TF"
                ),
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="True",
                description="Whether to launch RViz for SLAM inspection",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(
                    bringup_dir, "rviz", "nav2_default_view.rviz"
                ),
                description="Full path to the RViz config file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="True",
                description="Automatically startup lifecycle-managed SLAM nodes",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="False",
                description="Whether to respawn nodes if they crash",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            start_robot_state_publisher,
            slam_group,
            start_rviz,
        ]
    )

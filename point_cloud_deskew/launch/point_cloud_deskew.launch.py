from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("point_cloud_deskew"),
                        "config",
                        "point_cloud_deskew.yaml",
                    ]
                ),
                description="Full path to the point cloud deskew parameter file.",
            ),
            Node(
                package="point_cloud_deskew",
                executable="point_cloud_deskew_node",
                name="point_cloud_deskew",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )

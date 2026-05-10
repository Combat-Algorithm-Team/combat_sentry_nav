from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_point_cloud_fusion = LaunchConfiguration("use_point_cloud_fusion")
    use_odom_adapter = LaunchConfiguration("use_odom_adapter")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sentry_fusion"),
                        "config",
                        "sentry_fusion.yaml",
                    ]
                ),
                description="Full path to the point cloud deskew parameter file.",
            ),
            DeclareLaunchArgument(
                "use_point_cloud_fusion",
                default_value="True",
                description="Fuse the deskewed Livox cloud with the Odin cloud.",
            ),
            DeclareLaunchArgument(
                "use_odom_adapter",
                default_value="True",
                description="Convert Odin odometry into the navigation odometry outputs.",
            ),
            Node(
                package="sentry_fusion",
                executable="sentry_fusion_node",
                name="point_cloud_deskew",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "enable_fusion": ParameterValue(
                            use_point_cloud_fusion, value_type=bool
                        )
                    },
                ],
            ),
            Node(
                condition=IfCondition(use_odom_adapter),
                package="sentry_fusion",
                executable="odom_adapter_node",
                name="odom_adapter",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )

"""Starts the OAK-D driver and the AprilTag localizer, optionally with RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("rwrumi_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("tag_map", default_value=PathJoinSubstitution([share, "config", "tag_map.yaml"])),
            DeclareLaunchArgument("fps", default_value="15.0", description="Use 30.0 on a USB 3 connection"),
            DeclareLaunchArgument("ir_dot_intensity", default_value="0.0", description="IR dot projector, 0.0 to 1.0"),
            DeclareLaunchArgument("mxid", default_value="", description="Camera ID, empty for the first camera"),
            DeclareLaunchArgument("rviz", default_value="false"),
            Node(
                package="rwrumi_camera",
                executable="oakd_node",
                name="oakd",
                output="screen",
                parameters=[
                    {
                        "fps": ParameterValue(LaunchConfiguration("fps"), value_type=float),
                        "ir_dot_intensity": ParameterValue(LaunchConfiguration("ir_dot_intensity"), value_type=float),
                        "mxid": ParameterValue(LaunchConfiguration("mxid"), value_type=str),
                    }
                ],
            ),
            Node(
                package="rwrumi_pose",
                executable="tag_localizer_node",
                name="tag_localizer",
                output="screen",
                parameters=[{"tag_map": ParameterValue(LaunchConfiguration("tag_map"), value_type=str)}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([share, "rviz", "camera_localization.rviz"])],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )

# Launch the syncai_robot_3d_mapping node with the params file shipped in this package.
#
#   ros2 launch syncai_robot_3d_mapping mapping.launch.py
#   ros2 launch syncai_robot_3d_mapping mapping.launch.py namespace:=robot_1

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("syncai_robot_3d_mapping"),
        "params",
        "mapping_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Node namespace (usually the robot_id)",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Full path to the params YAML for the node",
            ),
            Node(
                package="syncai_robot_3d_mapping",
                executable="mapping_node",
                name="syncai_robot_3d_mapping",
                namespace=LaunchConfiguration("namespace"),
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
            ),
        ]
    )

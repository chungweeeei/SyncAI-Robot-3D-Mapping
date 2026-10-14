# Starts pgo_node (syncai_mapping) and nothing else; the mapping session runs it
# after pointlio.launch.py. Do not add a pointlio Node here: a drifted second
# copy once left the LIO unconfigured. The only coupling is the namespace.
#
# Namespace /<robot_id>/pgo (robot_id from the system INI) is load-bearing:
# pgo_node derives its inputs (/<robot_id>/pointlio/...), LIO reset client and
# local frame (<robot_id>/pointlio_odom) from it, so they are not parameters.
#
# Parameters come from params/mapping_params.yaml; this launch layers only
# map_cloud_dir (robot_id-dependent) and start_on_launch on top.

import configparser
import os

import launch
import launch_ros.actions
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# Absolute, like pointlio.launch.py and lio_bridge.launch.py, so it does not
# depend on the cwd.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

# Base of the map-cloud hand-off dir (/<robot_id> is appended). Must match the
# YAML's fallback and the path the backend reads.
DEFAULT_MAP_CLOUD_DIR = "/dev/shm/syncai_pgo"

logger = launch_logging.get_logger("mapping.launch")


def read_robot_id(config_path: str) -> str:
    config = configparser.ConfigParser()
    if not config.read(config_path):
        logger.warning(
            f"System config '{config_path}' not found; "
            f"falling back to robot_id '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    robot_id = config.get("system", "robot_id", fallback="").strip()
    if not robot_id:
        logger.warning(
            f"No [system] robot_id in '{config_path}'; "
            f"falling back to '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    return robot_id


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)
    map_cloud_base = LaunchConfiguration("map_cloud_dir").perform(context)
    # A launch argument is a string; the node declares a bool.
    start_on_launch = (
        LaunchConfiguration("start_on_launch").perform(context).strip().lower() == "true"
    )

    pkg_mapping = FindPackageShare("syncai_mapping").find("syncai_mapping")
    params_file = os.path.join(pkg_mapping, "params", "mapping_params.yaml")

    # After the params file, so they win.
    overrides = {
        # Per robot, so two robots on one host do not prune each other's files.
        "map_cloud_dir": f"{map_cloud_base}/{robot_id}",
        # Lets a bag replay skip pgo/start_mapping from the command line.
        "start_on_launch": start_on_launch,
    }

    # `name=` is safe: this process hosts a single node (unlike the planner /
    # controller, where it would rename their internal costmap too).
    return [
        launch_ros.actions.Node(
            package="syncai_mapping",
            namespace=f"{robot_id}/pgo",
            executable="pgo_node",
            name="pgo_node",
            output="screen",
            parameters=[params_file, overrides],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "system_config",
                default_value=DEFAULT_SYSTEM_INI,
                description="Path to the system INI file providing [system] robot_id",
            ),
            DeclareLaunchArgument(
                "map_cloud_dir",
                default_value=DEFAULT_MAP_CLOUD_DIR,
                description="Base tmpfs directory for the map-cloud hand-off; "
                "/<robot_id> is appended",
            ),
            DeclareLaunchArgument(
                "start_on_launch",
                default_value="false",
                description="true: map from the first pair instead of waiting for "
                "pgo/start_mapping (bag replays)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

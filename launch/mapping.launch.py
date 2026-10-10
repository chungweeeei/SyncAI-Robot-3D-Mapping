# Pose-graph-optimisation launch: starts pgo_node (syncai_mapping) and nothing
# else.
#
# This is THE definition of how pgo_node is configured. The mapping session
# (config/sessions/start_mapping.yaml) runs it in the `lio` window right after
# `ros2 launch syncai_pointlio pointlio.launch.py`, the front end it consumes.
# Until 2026-09 it lived in SyncAI-Fast-LIO2 as pgo_launch.py, and until
# earlier that month it also include()d pointlio's launch; that include existed
# because a second, drifted copy of the pointlio Node had once passed a
# `config_path` nobody read, so the LIO fell back to struct defaults and
# subscribed to topics nothing published — no odom, no body_cloud, nothing for
# pgo to optimise. Do not grow a pointlio Node definition here: the only
# coupling with pointlio.launch.py is the namespace below. pgo_node derives
# its inputs (/<robot_id>/pointlio/{body_cloud_dense,lio_odom}), its LIO reset
# client (/<robot_id>/pointlio/reset) and its local frame
# (<robot_id>/pointlio_odom) from running at /<robot_id>/pgo, so those names
# are not parameters and this launch does not pass them.
#
# robot_id is read from the system config INI at launch time (the workspace-
# wide convention) and is used as the namespace:
#   /<robot_id>/pgo/...   (start_mapping, save_maps, reset_mapping, map_cloud,
#                          map_cloud_file, mapping_status, loop_markers)
# The namespace is load-bearing: a pgo_node run anywhere other than
# /<robot_id>/pgo looks for pointlio in the wrong place.
#
# pgo_node's settings are plain ROS parameters (params/mapping_params.yaml,
# keyed by the `/**/pgo_node:` wildcard so the file works at any namespace),
# so this launch passes the installed file straight through and layers only
# map_cloud_dir (robot_id-dependent) and start_on_launch on top. It used to
# instead rewrite the whole YAML into /tmp/syncai_pgo/pgo_<robot_id>.yaml,
# because the node parsed that YAML itself with yaml-cpp and knew nothing
# about ROS parameters.

import configparser
import os

import launch
import launch_ros.actions
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# Absolute rather than the relative `config/system.ini` the nav packages'
# launch files use: identical to pointlio.launch.py and lio_bridge.launch.py,
# so the three LIO-side launches resolve the INI the same way whether or not
# the cwd is the workspace root.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

# Base of the per-robot tmpfs directory the merged map cloud is handed to the
# backend through (see map_cloud_dir in params/mapping_params.yaml). The
# launch appends /<robot_id>; this default has to agree with the YAML's
# fallback and with the path the backend's container reads.
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

    # Layered AFTER the params file so they win (later entries override
    # earlier ones).
    overrides = {
        # Per-robot subdirectory of the shared tmpfs the merged map cloud is
        # handed to the backend through. Two robots on one host must not
        # prune each other's files.
        "map_cloud_dir": f"{map_cloud_base}/{robot_id}",
        # Not robot_id-dependent; passed through so a bag replay can skip the
        # Start from the command line. The YAML's false is the session's value.
        "start_on_launch": start_on_launch,
    }

    # `name=` is fine here, unlike the planner / controller launches: this
    # process hosts exactly one node, so there is no second node to be
    # remapped onto the same name and lose its parameters.
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

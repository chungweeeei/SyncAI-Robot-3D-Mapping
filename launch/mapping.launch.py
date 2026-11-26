# Starts pgo_node (syncai_mapping) and nothing else; the mapping session runs it
# after pointlio.launch.py. Do not add a pointlio Node here: a drifted second
# copy once left the LIO unconfigured. The only coupling is the namespace.
#
# Namespace /<robot_id>/pgo (robot_id from the system INI) is load-bearing:
# pgo_node derives its inputs (/<robot_id>/pointlio/...), LIO reset client and
# local frame (<robot_id>/pointlio_odom) from it, so they are not parameters.
#
# Parameters come from params/mapping_params.yaml (what every robot shares),
# then params/models/<robot_model>.yaml (its body: map_clean_min_range,
# map_clean_lidar_height, map_clean_track_clear_height; robot_model from the INI, g23 when absent, and no
# model file is a refusal), then map_cloud_dir and resume_last_pose_file (both
# robot_id-dependent) and start_on_launch on top.

import configparser
import os

import yaml

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

# Every INI before [system] robot_model existed is a G23's (the Workspace's
# launches read the key the same way).
DEFAULT_ROBOT_MODEL = "g23"

# The keys a model's file must set: what differs with the robot's body. Not in
# the shared params file, so a model that left one out would run on the C++
# default -- the G23's -- and clean its maps around the wrong floor.
MODEL_KEYS = ("map_clean_min_range", "map_clean_lidar_height", "map_clean_track_clear_height")

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


# <workspace>/config/state/<robot_id>/last_pose.json: the file syncai_localizer
# saves its last trusted pose to, which a resume_mapping without a hint starts
# from. The same derivation as localizer.launch.py's last_pose_path (the
# workspace root is two directories above the INI), on purpose: the two must
# name one file, so change them together.
def last_pose_path(config_path: str, robot_id: str) -> str:
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(config_path)))
    return os.path.join(workspace_root, "config", "state", robot_id, "last_pose.json")


def read_robot_model(config_path: str) -> str:
    """[system] robot_model of the INI, `g23` when the key (or the file) is missing."""
    config = configparser.ConfigParser()
    config.read(config_path)
    model = config.get("system", "robot_model", fallback="").strip().lower()
    if not model:
        logger.warning(
            f"No [system] robot_model in '{config_path}'; reading it as '{DEFAULT_ROBOT_MODEL}'"
        )
        return DEFAULT_ROBOT_MODEL
    return model


def model_params_file(pkg_share: str, model: str) -> str:
    """params/models/<model>.yaml in this package's share; raises when it is missing,
    unreadable, or leaves out one of MODEL_KEYS.

    Nothing falls back, as in syncai_driver_manager and the Workspace's launches:
    another robot mapping on the G23's lidar height would clean its floor 10+ cm
    off and look healthy doing it. A new model's file needs a colcon build of this
    package before a launch finds it (--symlink-install links each file).
    """
    path = os.path.join(pkg_share, "params", "models", f"{model}.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(
            f"robot_model '{model}' has no params/models/{model}.yaml in syncai_mapping; "
            "add one (see params/models/g23.yaml) and rebuild, or fix [system] robot_model"
        )
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = (data.get("/**/pgo_node") or {}).get("ros__parameters") or {}
    except (OSError, yaml.YAMLError, AttributeError) as e:
        raise RuntimeError(f"robot_model '{model}': '{path}' is unreadable ({e})") from e
    missing = [k for k in MODEL_KEYS if k not in params]
    if missing:
        raise RuntimeError(
            f"robot_model '{model}': '{path}' leaves {missing} unset under /**/pgo_node "
            "(the C++ default is the G23's)"
        )
    logger.info(
        f"robot_model '{model}': {path} "
        + ", ".join(f"{k} {params[k]}" for k in MODEL_KEYS)
    )
    return path


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
    model_file = model_params_file(pkg_mapping, read_robot_model(config_path))

    # After the params file, so they win.
    overrides = {
        # Per robot, so two robots on one host do not prune each other's files.
        "map_cloud_dir": f"{map_cloud_base}/{robot_id}",
        # Lets a bag replay skip pgo/start_mapping from the command line.
        "start_on_launch": start_on_launch,
        # Read-only here; a missing file is "search the whole map".
        "resume_last_pose_file": LaunchConfiguration("last_pose_file").perform(context)
        or last_pose_path(config_path, robot_id),
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
            # The model's file under the same /**/pgo_node key as the shared
            # one, so it wins key by key; the overrides dict (/**) sets none of
            # the keys either file does.
            parameters=[params_file, model_file, overrides],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "system_config",
                default_value=DEFAULT_SYSTEM_INI,
                description="Path to the system INI file providing [system] robot_id "
                "and robot_model",
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
            DeclareLaunchArgument(
                "last_pose_file",
                default_value="",
                description="The localizer's last pose file a resume_mapping without a "
                "hint starts from; empty: <workspace>/config/state/<robot_id>/last_pose.json",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

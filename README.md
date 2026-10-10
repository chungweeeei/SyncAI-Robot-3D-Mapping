# SyncAI-Robot-3D-Mapping

The ROS 2 package `syncai_robot_3d_mapping`, written in Rust with
[`ros2_rust`](https://github.com/ros2-rust/ros2_rust) (`rclrs` 0.8), plus a Docker environment for
developing it on its own. Its layout and tooling follow SyncAI-Robot-State.

**This repo is itself a single colcon package** (`package.xml` / `Cargo.toml` live at the root), so
it can be pulled into SyncAI-Robot-Workspace with vcstool as `src/syncai_robot_3d_mapping`.

## The node

Right now it is a minimal skeleton: node `syncai_robot_3d_mapping` (executable `mapping_node`)
publishes a `std_msgs/String` heartbeat on the relative topic `heartbeat`.

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `frame_id` | string | `map` | Frame the map is built in |
| `heartbeat_rate` | double | `1.0` | Heartbeat publish rate, in Hz |

Both are read-only (load-time only); set them in `params/mapping_params.yaml` or via the launch file.

## Layout

```
.
├── Cargo.toml / package.xml     # the colcon package (ament_cargo)
├── src/
│   ├── main.rs                  # executor + node creation
│   └── mapping_node/
│       ├── mod.rs               # MappingNode: publisher, worker, timer
│       └── parameters.rs        # parameter declaration, Hz -> period
├── launch/mapping.launch.py
├── params/mapping_params.yaml
├── .devcontainer/               # ROS 2 Humble + Rust 1.85 + ros2_rust underlay
└── .github/                     # CI (fmt on the runner, build/clippy/test in the image)
```

## Development environment

Everything is built inside the Dev Container (`syncai-robot-3d-mapping`): open the repo in VS Code
and choose **Reopen in Container**, or run `devcontainer up --workspace-folder .`. The first image
build compiles the ros2_rust underlay from source and takes a while.

Inside the container, `/workspace` is a colcon workspace (named volume) and this repo is mounted at
`/workspace/src/syncai_robot_3d_mapping`.

```bash
cd /workspace && colcon build --symlink-install
source install/setup.bash
ros2 run syncai_robot_3d_mapping mapping_node
# or
ros2 launch syncai_robot_3d_mapping mapping.launch.py namespace:=robot_1

ros2 topic echo /heartbeat std_msgs/msg/String
```

Lint and test (the same commands CI runs, see `.github/scripts/ci.sh`):

```bash
cd /workspace/src/syncai_robot_3d_mapping
cargo fmt --check
cargo clippy --target-dir /workspace/build/.clippy --all-targets -- -D warnings
cargo test --target-dir /workspace/build/.clippy
```

The container uses the host network with `ROS_DOMAIN_ID=2`, the same as the other SyncAI dev
containers. Use `export ROS_DOMAIN_ID=77` when you want to stay off any shared graph.

# CLAUDE.md

The ROS 2 package `syncai_mapping` (C++17, `rclcpp`), the SyncAI mapping back end, moved here from
`~/Projects/SyncAI-Robot-Workspace/src/syncai_mapping` (source commit `c327059` there), with the
offline `hba_node` (HBA / BALM refinement) left behind: only the pose graph lives here. C++ rather
than rclrs because GTSAM and PCL are far more mature there. Two executables:

- `pgo_node`: pose graph over `syncai_pointlio`'s odometry and dense body clouds. Keyframes, loop
  closure (small_gicp + `PlanarLoopFactor`), GTSAM iSAM2, the `map -> <robot_id>/pointlio_odom`
  TF, and the `start_mapping` / `save_maps` / `reset_mapping` services plus `mapping_status`.
- `build_octomap`: the detached post-save OctoMap build that `pgo_node` spawns.

`README.md` is the authoritative behaviour reference (the run lifecycle, threading, loop
verification, the OctoMap build, gotchas). Read the relevant section before changing behaviour.

**The external interface is a cross-repo contract**: the package name `syncai_mapping`, executable
names, node names / namespaces, topics, services (typed by `syncai_common`), parameters and the
`/dev/shm/syncai_pgo/<robot_id>` hand-off are used by SyncAI-Robot-Workspace's session specs and
by the backend. Do not rename any of them casually.

**The repo root is the package itself** (`package.xml` and `CMakeLists.txt` are at the root), meant
to be vcs-imported into the Workspace as `src/syncai_mapping`. `.devcontainer/` and
`dependencies.repos` are only for developing this repo on its own.

## Build and test

Everything runs in the Dev Container `syncai-robot-3d-mapping` (VS Code Reopen in Container, or
`devcontainer up --workspace-folder .`). `/workspace` is the named volume
`syncai-robot-3d-mapping_workspace`, this repo is mounted at `/workspace/src/syncai_mapping`, and
`postCreateCommand` vcs-imports `dependencies.repos` (`syncai_common`, `small_gicp`) next to it.
The host has no ROS, GTSAM or PCL: nothing compiles there.

```bash
docker exec syncai-robot-3d-mapping bash -lc 'cd /workspace && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
docker exec syncai-robot-3d-mapping bash -lc 'cd /workspace && colcon test --packages-select syncai_mapping && colcon test-result --test-result-base build/syncai_mapping --verbose'
```

- If `syncai_common` / `small_gicp` are missing (a fresh volume):
  `cd /workspace && vcs import --skip-existing < src/syncai_mapping/dependencies.repos`.
- CI (`.github/workflows/ci.yml`) runs a clang-format 14 check on an ubuntu-22.04 runner and
  `.github/scripts/ci.sh` (vcs import, colcon build, colcon test with ament_lint_auto) inside the
  image built from `.devcontainer/Dockerfile`. Keep the script and this list in step.
- Stop a test node with `pkill -x pgo_node` (not `-f`, which also kills the `docker exec` shell).

## Dependencies

- **GTSAM 4.2.0 is source-built into `/usr/local`** by the Dockerfile with the same flags as the
  Workspace image (system Eigen, no march-native, TBB on, shared). Keep them in step with the
  Workspace: a GTSAM built with its bundled Eigen or `-march=native` causes Eigen-alignment crashes
  when mixed with PCL. It has no `<depend>` in `package.xml` on purpose.
- PCL, Eigen and OctoMap come from apt (`ros-humble-pcl-ros`, `libeigen3-dev`,
  `ros-humble-octomap`).
- `syncai_common` and `small_gicp` are colcon packages from `dependencies.repos`; keep those pins in
  step with the Workspace's `interface.repos` / `third-party.repos`.

## Real-robot safety

The dev container uses the host network and `ROS_DOMAIN_ID=2`. `pgo_node`'s `start_mapping` /
`reset_mapping` call `syncai_pointlio`'s `ResetLIO`, and it broadcasts the `map` TF. When replaying
bags or testing, `export ROS_DOMAIN_ID=77` so nothing reaches a real robot's graph.

## Conventions

- Everything is written in English (code, comments, logs, docs). Log messages carry a `[Module]`
  prefix (e.g. `[PGONode]`, `[saveMapsCB]`).
- Formatting follows `.clang-format` (ROS 2 / Google based, copied from SyncAI-Robot-Workspace).
- `pgos/` is upstream-derived (FASTLIO2_ROS2 lineage) and MIT-licensed (`LICENSE`).
- Commits use Conventional Commits (`.github/prompt/copilot-commit-message-instructions.md`), in
  English.

# SyncAI-Robot-3D-Mapping

The ROS 2 package **`syncai_mapping`** (C++17, `rclcpp`): SyncAI's mapping back end. The repo root
is the package itself, pulled into SyncAI-Robot-Workspace with vcstool as `src/syncai_mapping`.

It has two executables:

- **`pgo_node`**: a pose graph over `syncai_pointlio`'s odometry and dense body clouds. It picks
  keyframes, closes loops (small_gicp GICP + GTSAM iSAM2), broadcasts the
  `map → <robot_id>/pointlio_odom` correction, streams the "map so far" to the operator console,
  and serves `start_mapping`, `save_maps` and `reset_mapping`. It runs only in the mapping
  session; in navigation `syncai_localizer` owns the same TF.
- **`clean_map`**: a detached job `pgo_node` starts after each save. It rewrites the saved
  `map.pcd` without people and one-off returns.

```
   syncai_pointlio            /<id>/pointlio/{body_cloud_dense, lio_odom}
        │                           │  (ApproximateTime sync)
        │ reset ◄────── ResetLIO ───┤
        │                           ▼
        │                       pgo_node                  /<id>/pgo/…
        │             keyframes · loop closure · iSAM2
        │             TF  map ──► <id>/pointlio_odom
        │                ┌──────────┼──────────────┬────────────────┬──────────────┐
        │                ▼          ▼              ▼                ▼              ▼
        │          map_cloud   map_cloud_file  save_maps      start_mapping   mapping_status
        │          (rviz)      JSON notice ──► map/<name>/    reset_mapping   IDLE / MAPPING /
        │                      /dev/shm/…      (ends the run)                 RESETTING
        │                      (backend preview)    │
        │                                           ▼
        │                                       clean_map ──► map.pcd (cleaned)
```

Source layout: `src/pgos/` is the pose graph (upstream-derived, MIT), `src/pgo_node.cpp` is the
ROS node, and `src/dynamic_removal/` is `clean_map`.

Paths below such as `~/robot_ws`, `config/system.ini` and `map/<name>` refer to the robot's
SyncAI-Robot-Workspace deployment.

## Inputs

| Topic | Type | Use |
|---|---|---|
| `/<robot_id>/pointlio/body_cloud_dense` | `sensor_msgs/PointCloud2` | the keyframe scan, body frame (x / y / z / intensity) |
| `/<robot_id>/pointlio/lio_odom` | `nav_msgs/Odometry` | the keyframe pose in `<robot_id>/pointlio_odom` |

These names, the LIO reset client `/<robot_id>/pointlio/reset` and the frames are **not
parameters**. The node derives them from its own namespace (`/<robot_id>/pgo`), so it must run
there. It logs the resolved names at startup.

The two topics are paired by an `ApproximateTime` synchroniser. Only the newest pair is kept, and a
50 ms timer takes it: under load whole frames are skipped rather than queued. While the node is
idle, pairs are dropped and an identity `map → local_frame` is broadcast so the console's live scan
stays visible.

## Outputs

All under `/<robot_id>/pgo/` except the TF.

| Output | Type | Notes |
|---|---|---|
| TF `map → <robot_id>/pointlio_odom` | | the loop-closure correction, on every pair |
| `map_cloud` | `PointCloud2` | the "map so far" for rviz |
| `map_cloud_file` | `std_msgs/String`, reliable + transient local | JSON notice naming the same map as a PCD file; what the backend reads |
| `loop_markers` | `MarkerArray` | loop nodes and edges for rviz |
| `mapping_status` | `syncai_common/msg/MappingStatus`, reliable + transient local | `IDLE` / `MAPPING` / `RESETTING` plus keyframe and loop counts |
| `start_mapping` | `syncai_common/srv/StartMapping` | begin a run |
| `save_maps` | `syncai_common/srv/SaveMaps` | write the map and end the run |
| `reset_mapping` | `syncai_common/srv/ResetMapping` | discard the run and start over |

Both map outputs are published only when someone subscribes, after a new keyframe, and at most
every `map_cloud_pub_period` (3 s).

### Map-cloud file hand-off

A large map is 16–45 MB, too big to deliver reliably over DDS on `lo`. So the node writes it as a
binary PCD to `<map_cloud_dir>/map_cloud_<seq>.pcd` (keeping the newest two) and publishes a small
notice:

```json
{"seq":12,"path":"/dev/shm/syncai_pgo/robot01/map_cloud_12.pcd","points":1830412,"frame_id":"map","stamp":{"sec":1727,"nanosec":0}}
```

- `map_cloud_dir` is `/dev/shm/syncai_pgo/<robot_id>`. The backend can read it only if **both
  compose services run with `ipc: host`**.
- Subscribe transient local, or a restarted backend misses the latest notice.
- Start, reset and a successful save publish an empty map (`"points":0,"path":""`). Consumers must
  clear their view on it.
- If the directory cannot be created, only the file output is disabled; the node keeps running.

### `save_maps`

`file_path` must be an existing directory. The node writes:

| File | Content |
|---|---|
| `map.pcd` | all keyframes in their corrected global poses, binary, not voxel-filtered; later replaced by `clean_map`'s output |
| `patches/<i>.pcd` | (with `save_patches`) one body-frame cloud per keyframe |
| `poses.txt` | (with `save_patches`) `<i>.pcd tx ty tz qw qx qy qz` per keyframe |
| `map_clean.recipe.json` | (with `save_patches` and `map_clean_enabled`) the cleaning status |

No file names the map or holds an absolute path, so a map directory can be renamed freely.

**A successful save ends the run**: the node frees the keyframes, publishes the empty map and goes
`IDLE`. A failed save changes nothing. Keyframes live only in RAM, so anything not saved before
the process exits or a reset is lost.

### Cleaning map.pcd

After a save with patches, `pgo_node` spawns `clean_map` on the saved directory. It ray-casts every
keyframe through 0.1 m voxels, counting for each voxel how many keyframes **hit** it (H) and how
many **passed through** it (M), and rewrites `map.pcd` without two classes of points:

| Class | Rule | Catches |
|---|---|---|
| dynamic | `M >= dynamic_min_miss` (3) and `M >= dynamic_miss_ratio × H` (2.0), between floor + `floor_band` and floor + `dynamic_max_height` | people |
| sparse | `H < sparse_min_hits` (2), outside the floor band | smear, edge returns, reflections |

- **Offline on purpose.** A person shows up only once a later scan sees through them, and the
  judgement needs the final loop-closed poses. Until the clean finishes, `map.pcd` is the raw save.
- **Input is `patches/` + `poses.txt`**, not `map.pcd`, because rays need each scan's origin. No
  patches, no clean. `patches/` stay raw, so a rerun is idempotent.
- **The floor band is exempt** from both rules (the floor is sampled thinly). The floor height is
  taken per level from nearby keyframes, within `floor_radius`.
- **Rays stop at the floor** (`clip_below_floor`), so reflections under a glossy floor do not
  carve away voxels below it.
- **Output is PCD only**: `map.pcd` and the recipe. OctoMap is used only for voxel keys and ray
  traversal; no octree or OctoMap file is produced.
- **Process model.** A detached child process (`posix_spawn`, own session, `nice`), so it survives
  a mode switch and the OOM killer picks it before `pgo_node`. One clean runs per host (a `flock` on
  `/dev/shm/syncai_pgo/clean_map.lock`). A new save into the same directory stops the old clean.

**Status** is `map_clean.recipe.json`: `converting` → `ok` / `failed`, with `started_at`,
`finished_at`, the `params` used and, on `ok`, the `measurements`. A hard kill leaves it at
`converting` with the raw map. **A 2D map made while it says `converting` came from the raw map**;
redo it once it reads `ok`.

Run by hand to re-clean any save with patches:

```bash
ros2 run syncai_mapping clean_map map/<name>
ros2 run syncai_mapping clean_map map/<name> --lock none --dynamic-min-miss 0   # sparse rule only
#   --voxel-size 0.1 --max-range 20 --min-range 0.5 --lidar-height 0.481 --floor-band 0.25
#   --dynamic-max-height 2.0 --floor-radius 10 --sparse-min-hits 2 --dynamic-miss-ratio 2.0
#   --dynamic-min-miss 3 --clip-below-floor 1 --nice 0 --started-at <ISO-8601>
# exit 0 ok · 1 usage · 2 no poses.txt / readable patch · 3 failed · 4 interrupted
cat map/<name>/map_clean.recipe.json
```

### Run lifecycle

The node starts `IDLE` (or `MAPPING` with `start_on_launch`) and reports its state on
`mapping_status`.

| State | Event | Next |
|---|---|---|
| `IDLE` | `start_mapping` | `RESETTING` → `MAPPING` |
| `IDLE` | `reset_mapping` / `save_maps` | refused |
| `MAPPING` | `reset_mapping` | `RESETTING` → `MAPPING` |
| `MAPPING` | `start_mapping` | refused ("already mapping") |
| `MAPPING` | `save_maps` succeeds / fails | `IDLE` / `MAPPING` |
| `RESETTING` | anything | pairs dropped, services refused |

Start and reset share one sequence:

1. **Pause**: check the precondition and switch to `RESETTING`, so no pair reaches the graph.
2. **Reset pointlio** (`reset_lio: true`): call `ResetLIO` and wait up to 5 s. On failure, nothing
   has changed yet and the call returns an error.
3. **Rebuild**: replace the pose graph with a fresh one, publish the empty map, switch to
   `MAPPING`.
4. **Resume**: drop pairs stamped at or before pointlio's last old odometry, so the old run's tail
   cannot leak in.

**The robot must stand still** for start and reset: pointlio re-runs its gravity-aligning IMU
initialisation, and doing that in motion tilts the whole map without any error. Nothing enforces
this, so the service responses say it. `reset_lio: false` resets only the graph, for bag replay.

### Threading

`src/main.cpp` runs a 3-thread `MultiThreadedExecutor`:

- the default group: the timer, both subscriptions, the status timer and `save_maps`, serialised;
- a second group: `start_mapping` and `reset_mapping`, which block on the `ResetLIO` call;
- a third group: the `ResetLIO` client, so its response arrives while the caller waits.

`m_pgo_mutex` guards the pose graph; the timer and `save_maps` hold it for their whole body, start
and reset only around the rebuild (never while waiting on pointlio). The run state and counters are
atomics. The map merge runs on its own thread from a copy of the keyframes.

## Parameters

From `params/mapping_params.yaml` (keyed `/**/pgo_node:`). Read once at startup and on every
reset; `ros2 param set` afterwards has no effect. A missing or misspelt key falls back to the
default silently.

| Parameter | Default | Notes |
|---|---|---|
| `map_cloud_dir` | `/dev/shm/syncai_pgo` | the launch appends `/<robot_id>` |
| `start_on_launch` | `false` | start `MAPPING` without a `start_mapping` call (bag replay) |
| `keyframe_delta_deg` / `keyframe_delta_m` | `10.0` / `0.5` | a new keyframe every 10° or 0.5 m |
| `loop_search_radius` | `1.0` | m, radius for loop candidates |
| `loop_time_tresh` | `60.0` | s, minimum age of a candidate |
| `loop_score_tresh` | `0.15` | GICP fitness (m²) above this is rejected |
| `loop_search_period` | `5.0` | s between loop searches |
| `loop_submap_half_range` | `5` | **int**, keyframes on each side of the candidate |
| `loop_submap_resolution` | `0.1` | m, voxel size of the GICP submap |
| `loop_gicp_max_corr_dist` | `1.0` | m, max GICP correspondence distance |
| `loop_gicp_num_threads` / `loop_gicp_num_neighbors` | `4` / `20` | **ints** |
| `loop_planar_correction` | `true` | loop edge constrains only world x / y / yaw (`PlanarLoopFactor`); `false` = 6-DOF `BetweenFactor` |
| `loop_noise_xy_sigma_m` / `loop_noise_yaw_sigma_deg` | `0.05` / `1.5` | planar loop sigmas; `<= 0` = use the fitness as variance |
| `loop_noise_var_roll_pitch_z` | `0.01` | only with planar correction off |
| `keyframe_tilt_sigma_deg` | `0.5` | holds each keyframe's roll / pitch to pointlio's; `<= 0` = off |
| `map_cloud_resolution` | `0.2` | m, voxel size of the published map |
| `map_cloud_pub_period` | `3.0` | s, minimum gap between map publishes |
| `map_clean_enabled` | `true` | spawn `clean_map` after a save with patches |
| `map_clean_voxel_size` | `0.1` | m |
| `map_clean_max_range` / `map_clean_min_range` | `20.0` / `0.5` | m, ray length / robot-body cut-off |
| `map_clean_lidar_height` | `0.481` | m, lidar above the floor |
| `map_clean_floor_band` | `0.25` | m, band around the floor exempt from both rules |
| `map_clean_floor_radius` | `10.0` | m, keyframes within this set a cell's floor |
| `map_clean_dynamic_max_height` | `2.0` | m, top of the people window |
| `map_clean_dynamic_miss_ratio` / `map_clean_dynamic_min_miss` | `2.0` / `3` | dynamic rule; `min_miss` is an **int**, `<= 0` = off |
| `map_clean_sparse_min_hits` | `2` | **int**, sparse rule; `<= 1` = off |
| `map_clean_clip_below_floor` | `true` | stop rays at the floor |
| `map_clean_nice` | `10` | **int**, the clean's nice value |

Every value is a double unless marked int. **Write doubles with a decimal point** (`10.0`): a bare
`10` for a double, or `5.0` for an int, kills the node at startup.

### Loop verification

A candidate (a past keyframe within `loop_search_radius`, at least `loop_time_tresh` old) is
verified by GICP-registering the current keyframe onto an 11-keyframe submap around it.

- **GICP with a 1 m correspondence distance.** The original PCL ICP (10 m) returned degrees of
  false rotation on corridor loops, and its fitness could not tell good loops from bad.
- **Planar loop edge.** A 6-DOF loop edge bent the chain and produced a double floor; the
  `PlanarLoopFactor` corrects only world x / y / yaw and leaves z / roll / pitch to the
  gravity-aligned odometry.
- **Logged.** Every accepted loop prints two `[PGONode][loop]` lines with the correction and the
  keyframes' z before and after.

Known gap: the fitness gate counts all source points, so it still cannot reliably reject a bad
corridor loop.

## Running

```bash
ros2 launch syncai_mapping mapping.launch.py                                      # config/system.ini from ~/robot_ws
ros2 launch syncai_mapping mapping.launch.py system_config:=/path/to/robot01.ini
ros2 launch syncai_mapping mapping.launch.py map_cloud_dir:=/dev/shm/syncai_pgo   # /<robot_id> is appended
ros2 launch syncai_mapping mapping.launch.py start_on_launch:=true                # bag replay
```

This starts no LIO: run `syncai_pointlio` first. In the mapping session it is the second pane of
the `lio` window in `start_mapping.yaml`.

Without a backend:

```bash
ros2 service call /<robot_id>/pgo/start_mapping syncai_common/srv/StartMapping "{reset_lio: true}"   # robot still
ros2 service call /<robot_id>/pgo/save_maps syncai_common/srv/SaveMaps \
  "{file_path: '/home/syncrobotic/robot_ws/map/<name>', save_patches: true}"   # directory must exist
ros2 service call /<robot_id>/pgo/reset_mapping syncai_common/srv/ResetMapping "{reset_lio: true}"   # robot still
```

Checking it:

```bash
ros2 topic echo /<robot_id>/pgo/mapping_status --qos-durability transient_local --qos-reliability reliable
ros2 topic echo /<robot_id>/pgo/map_cloud_file --qos-durability transient_local --qos-reliability reliable
ros2 run tf2_ros tf2_echo map <robot_id>/pointlio_odom   # identity while idle
ls /dev/shm/syncai_pgo/<robot_id>                        # newest two map_cloud_<seq>.pcd; empty while idle
```

| Log line | Meaning |
|---|---|
| `[PGONode] inputs: /robot01/pointlio/body_cloud_dense + …` | names derived from the namespace; no `/robot01` prefix means the node is not under `/<robot_id>/pgo` |
| `[PGONode] idle until start_mapping is called; …` | normal at launch |
| `[PGONode][loop] target 44 -> source 252 \| fitness … \| GICP moves source by …` | a loop was accepted |
| `[saveMapsCB] Map saved (N keyframes). Mapping stopped …` | the run ended |
| `LIO reset service … is not available` / `Timed out waiting for the LIO reset` | start or reset refused: pointlio is down |
| `[PGONode] cannot create map_cloud_dir …` | the file hand-off is off for this run; the node keeps running |

## Gotchas

- **The ROS interface is a cross-repo contract.** The backend calls the three services, reads
  `mapping_status`, `map_cloud_file` and `/dev/shm/syncai_pgo/<robot_id>`, and expects the
  `map/<name>/` layout. Change none of it without updating the other side.
- **The node comes up idle.** A session that never calls `start_mapping` maps nothing.
- **`<robot_id>/pointlio_odom` must match pointlio's `world_frame`.** If pointlio's ever changes,
  update `PGONode::loadParameters()`, or the correction lands on a frame nobody uses.
- **GTSAM 4.2.0 is not in `package.xml`.** It is source-built into `/usr/local` by the Dockerfile
  (system Eigen, no `-march=native`); a GTSAM with bundled Eigen crashes alongside PCL.
- **Build in Release.** The CMake default; a debug build cannot keep up on the Jetson.
- **`cpplint` and `ament_cmake_copyright` are off**, because `pgos/` has no per-file licence
  headers (the package `LICENSE` covers it).

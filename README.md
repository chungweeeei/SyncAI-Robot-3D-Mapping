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
  `map.pcd` without people and one-off returns, and with `map_clean_octomap` (off by default)
  writes `octomap.bt`, the occupancy tree `octomap_planner` plans 3D global paths on.

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
        │                                       clean_map ──► map.pcd (cleaned), [octomap.bt]
```

Source layout: `src/pgos/` is the pose graph (upstream-derived, MIT), `src/pgo_node_impl.cpp` is
the ROS node, `src/dynamic_removal/` is the map cleaning and the ground patches, and `src/apps/`
holds one entry point per executable, named after it.

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
every `map_cloud_pub_period` (3 s). They are a preview: each keyframe is voxelised to
`map_cloud_resolution` once, at insertion, and the merge stacks those copies, re-voxelising every
2 M points, so it never holds a second dense copy of the run.

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
| `map.pcd` | all keyframes in their corrected global poses, binary, not voxel-filtered; later replaced by `clean_map`'s output. Streamed one keyframe at a time through `map.pcd.save.tmp` and renamed: the merge is never held in memory, and a failed save leaves the previous file whole |
| `patches/<i>.pcd` | (with `save_patches`) one body-frame cloud per keyframe |
| `poses.txt` | (with `save_patches`) `<i>.pcd tx ty tz qw qx qy qz` per keyframe |
| `map_clean.recipe.json` | (with `save_patches` and `map_clean_enabled`) the cleaning status |
| `octomap.bt` | written later by `clean_map` (with `map_clean_octomap`); a save deletes the previous one first |
| `octomap_edits.json` | not written: the ground patches the console's editor saved (see Ground patches); a save leaves it |
| `octomap.raw.bt` | not written: `apply_octomap_edits`'s copy of the tree before patching; a save or a new tree deletes it |

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
| dynamic | `M >= dynamic_min_miss` (3) and `M >= dynamic_miss_ratio × H` (2.0), between floor + `floor_band_dynamic` (0.1) and floor + `dynamic_max_height` | people, their feet included |
| sparse | `H < sparse_min_hits` (2), outside the floor band | smear, edge returns, reflections |

- **Offline on purpose.** A person shows up only once a later scan sees through them, and the
  judgement needs the final loop-closed poses. Until the clean finishes, `map.pcd` is the raw save.
- **Input is `patches/` + `poses.txt`**, not `map.pcd`, because rays need each scan's origin. No
  patches, no clean. `patches/` stay raw, so a rerun is idempotent.
- **The floor band is exempt** from the sparse rule (the floor is sampled thinly), and the dynamic
  rule starts only `floor_band_dynamic` above the floor, so feet are judged. The floor height is
  taken per level from nearby keyframes, within `floor_radius`: the highest level at or below the
  voxel. Where the voxel could be a person on the nearest keyframe's level (up to
  `dynamic_max_height` above it), only levels with a keyframe within 3 m of the nearest one's
  distance count, so a stair landing metres away does not become the floor of the corridor beside
  it.
- **Only points within `max_range` (20 m) are judged.** Farther points are kept, not removed. A
  larger value judges sparse, far returns and long, error-prone rays, which removes real
  structure; people near the path are judged from closer keyframes anyway.
- **Rays stop at the floor** (`clip_below_floor`), so reflections under a glossy floor do not
  carve away voxels below it.
- **`octomap.bt`** (`map_clean_octomap`): the same scans in an `OcTree` at `voxel_size`, free space
  as well as occupied, since a planner must tell observed-free from never-seen. It is filled in
  the misses pass, which already ray-casts every scan. Space no ray reached (behind walls, under
  the floor) has no node, i.e. is **unknown**. It agrees with the cleaned `map.pcd`: a removed
  voxel is free if a ray passed through it (where a person stood) and unknown if none did (a
  reflection outside a window). **A hit floor voxel stays occupied**, like the floor band's
  exemption from the cleaning: the lidar sees the floor only at grazing angles, and the rays to
  far floor points otherwise carve the near floor free (under the path only 31 % of the floor
  columns were occupied on dp1f_1008_2_v2; 96 % with this). On dp1f_1008_2_v2 (2004 keyframes) it took the clean from
  142 s to 208 s and its peak memory from 1.0 to 2.0 GB, for a 2.8 MB file. A tree that runs out of
  memory is dropped and `map.pcd` is still cleaned.
- **Floor fill** (`floor_fill_radius`, 3 m): even so, the floor 1-3 m from the path has gaps
  (the shadows of shelves and walls, sampled too thinly), and a legged robot's planner needs ground
  under every step. The tree (not `map.pcd`) gets those gaps closed at the height of the floor next
  to them: a column within that radius of a keyframe on the same level, **enclosed** (hit floor or
  an obstacle within 0.6 m on both sides along some line, so a pool or pit wider than 1.2 m stays a
  hole), with space up to 0.5 m above seen free and empty and nothing hit up to 1 m below (a stair down, a drop). Enclosed, not grown outward:
  grown outward it ran through doors, windows and glass fronts into fans of floor outside the
  building. On dp1f_1008_2_v2 it adds 43 k voxels; of the columns with open space above, 89 % (1-2
  m from the path) and 71 % (2-3 m) then have floor, from 72 % and 40 %. A hole no ray looked into
  is not caught; hence the radius.
- **Process model.** A detached child process (`posix_spawn`, own session, `nice`), so it survives
  a mode switch and the OOM killer picks it before `pgo_node`. One clean runs per host (a `flock` on
  `/dev/shm/syncai_pgo/clean_map.lock`). A new save into the same directory stops the old clean.

**Status** is `map_clean.recipe.json`: `converting` → `ok` / `failed`, with `started_at`,
`finished_at`, the `params` used and, on `ok`, the `measurements`; `octomap_written` says whether
`octomap.bt` came with it (`octomap_error` says why not). A hard kill leaves it at
`converting` with the raw map. **A 2D map made while it says `converting` came from the raw map**;
redo it once it reads `ok`.

Run by hand to re-clean any save with `patches/` + `poses.txt`; with `--octomap 1` this is also
how to convert one to `octomap.bt` offline. It rewrites `map.pcd` too, identically for the same
params (it re-cleans from the raw patches). A bare `map.pcd` cannot be converted: without scan
origins there is no free space.

```bash
ros2 run syncai_mapping clean_map map/<name>
ros2 run syncai_mapping clean_map map/<name> --lock none --dynamic-min-miss 0   # sparse rule only
ros2 run syncai_mapping clean_map map/<name> --octomap 1                        # also octomap.bt
#   --voxel-size 0.1 --max-range 20 --min-range 0.5 --lidar-height 0.481 --floor-band 0.25 --floor-band-dynamic 0.1
#   --dynamic-max-height 2.0 --floor-radius 10 --sparse-min-hits 2 --dynamic-miss-ratio 2.0
#   --dynamic-min-miss 3 --clip-below-floor 1 --octomap 0 --floor-fill-radius 3 --nice 0
#   --started-at <ISO-8601>
# exit 0 ok · 1 usage · 2 no poses.txt / readable patch · 3 failed · 4 interrupted
cat map/<name>/map_clean.recipe.json
```

### Ground patches

The floor fill closes only enclosed gaps, and a floor no ray looked into stays unknown, so the
planner refuses it — a corridor the lidar only glanced at, the far side of a hall. An operator who
knows it is floor paints it in the console's ground editor, which the backend records as
`map/<name>/octomap_edits.json`: absolute OcTree keys (16 bits an axis, the map origin at 32768)
of the voxels to make floor, one per xy column, with the resolution they were painted on. The
standalone viewer's export has the same shape. `apply_octomap_edits` is what puts them in the tree:

```bash
ros2 run syncai_mapping apply_octomap_edits map/<name>            # [--clearance 0.6] [--edits octomap_edits.json]
# exit 0 ok · 1 usage · 2 no octomap.bt / no record / unreadable record · 3 failed
```

- Each key becomes an occupied voxel, and the `clearance` (0.6 m, the planner's `body_clearance`)
  above it is marked observed-free — the planner rejects a floor with unknown space over it
  (`no_free_above` / `clearance_unknown`). A voxel already occupied in that band is left: a wall
  foot or a shelf, which the planner then refuses on its own, rightly.
- **It starts from `octomap.raw.bt` every time.** The first apply copies the clean's tree there;
  later ones read it and rewrite `octomap.bt` (`.tmp` + rename), so the result is a function of
  the record alone, a patch erased in the console really goes, and a rerun is idempotent. A new
  save and a new tree from `clean_map` both delete the copy, since the old one would describe the
  previous tree; `octomap_edits.json` stays, like `keepout.json` — the keys are that save's map
  frame, so after a new mapping run into the same directory they are the operator's to redraw.
- The record's `res` must be the tree's (exit 2 otherwise): keys index cells.
- The planner loads the tree once at start, so a patched map needs the navigation session
  restarted to take effect. Not yet: `clean_map` applying the record itself when it writes a new
  tree, which would keep the patches through a re-clean.

`test/test_octomap_edits.cpp` covers reading the record and these rules on small synthetic trees.

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

`src/apps/pgo_node.cpp` runs a 3-thread `MultiThreadedExecutor`:

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
| `loop_score_tresh` | `0.15` | mean squared distance (m²) over the overlap above this is rejected |
| `loop_search_period` | `5.0` | s between loop searches |
| `loop_submap_half_range` | `5` | **int**, keyframes on each side of the candidate |
| `loop_submap_resolution` | `0.1` | m, voxel size of the GICP submap |
| `loop_gicp_max_corr_dist` | `1.0` | m, max GICP correspondence distance |
| `loop_gicp_num_threads` / `loop_gicp_num_neighbors` | `4` / `20` | **ints** |
| `loop_planar_correction` | `true` | loop edge constrains only world x / y / yaw (`PlanarLoopFactor`); `false` = 6-DOF `BetweenFactor` |
| `loop_noise_xy_sigma_m` / `loop_noise_yaw_sigma_deg` | `0.05` / `1.5` | planar loop sigmas; `<= 0` = use the fitness as variance |
| `loop_noise_var_roll_pitch_z` | `0.01` | only with planar correction off |
| `keyframe_tilt_sigma_deg` | `0.5` | holds each keyframe's roll / pitch to pointlio's; `<= 0` = off |
| `loop_robust_kernel` / `loop_robust_kernel_scale` | `huber` / `5.0` | **string** `huber` / `cauchy` / `none`, scale in loop-noise sigmas |
| `loop_inlier_dist` | `0.2` | m, an overlap point this close to the submap is an inlier |
| `loop_min_overlap_ratio` / `loop_min_inlier_ratio` | `0.3` / `0.6` | overlap / source points, inliers / overlap points |
| `loop_min_xy_info_ratio` | `0.0` | corridor degeneracy gate (GICP x / y information eigenvalue ratio); `<= 0` = off, still logged |
| `loop_max_drift_ratio` | `0.05` | a loop may move the keyframe by at most this fraction of the path between its keyframes; `<= 0` = off |
| `loop_radius_drift_scaled` | `true` | radius search reaches as far as the drift gate allows (capped at `loop_search_radius` + 3 × `loop_gicp_max_corr_dist`); `false` = `loop_search_radius` only |
| `loop_max_candidates` | `3` | **int**, radius candidates verified per search |
| `loop_sc_enabled` | `false` | Scan Context global candidates |
| `loop_sc_max_range` / `loop_sc_dist_tresh` | `20.0` / `0.2` | m, descriptor radius / descriptor distance gate |
| `map_cloud_resolution` | `0.2` | m, voxel size of the published map |
| `map_cloud_pub_period` | `3.0` | s, minimum gap between map publishes |
| `map_clean_enabled` | `true` | spawn `clean_map` after a save with patches |
| `map_clean_voxel_size` | `0.1` | m |
| `map_clean_max_range` / `map_clean_min_range` | `20.0` / `0.5` | m, judging range / robot-body cut-off. Points outside are not judged and stay in `map.pcd`. `max_range` is not the lidar's range; keep it below it |
| `map_clean_lidar_height` | `0.481` | m, lidar above the floor |
| `map_clean_floor_band` | `0.25` | m, band around the floor exempt from both rules |
| `map_clean_floor_band_dynamic` | `0.1` | m, the dynamic rule's own exemption: above this the rule applies inside `floor_band` too. People's feet and ankles sit 0.1–0.25 m up and stayed in the map at every stop (dp1f_1012: the operator's, seen once from 3 m ahead, passed through 30–40 times); the floor itself lies within a voxel of the local floor |
| `map_clean_floor_radius` | `10.0` | m, keyframes within this set a cell's floor |
| `map_clean_dynamic_max_height` | `2.0` | m, top of the people window |
| `map_clean_dynamic_miss_ratio` / `map_clean_dynamic_min_miss` | `2.0` / `3` | dynamic rule; `min_miss` is an **int**, `<= 0` = off |
| `map_clean_sparse_min_hits` | `2` | **int**, sparse rule; `<= 1` = off |
| `map_clean_clip_below_floor` | `true` | stop rays at the floor |
| `map_clean_octomap` | `false` | also write `octomap.bt` |
| `map_clean_floor_fill_radius` | `3.0` | m, infer floor in `octomap.bt` this far from the path; `<= 0` = off |
| `map_clean_nice` | `10` | **int**, the clean's nice value |

Every value is a double unless marked int or string (the `true` / `false` switches are bools). **Write doubles with a decimal point** (`10.0`): a bare
`10` for a double, or `5.0` for an int, kills the node at startup.

### Loop verification

Each search (every keyframe, at most every `loop_search_period` after an accepted loop) collects
candidates and GICP-registers the current keyframe onto an 11-keyframe submap around each:

- **Radius candidates.** Past keyframes at least `loop_time_tresh` old, within
  `loop_search_radius` of the current estimate in z, and in xy within what the drift gate would
  accept for that pair: `max(loop_search_radius, loop_max_drift_ratio × path)`, capped at
  `loop_search_radius` + 3 × `loop_gicp_max_corr_dist`, the coarse stage's reach
  (`loop_radius_drift_scaled`). Nearest first, up to `loop_max_candidates`, each more than a
  submap apart; the one with the best inlier ratio wins. One beyond `loop_search_radius` is
  registered coarse to fine like a Scan Context one. A fixed 1 m radius never saw a revisit that
  had drifted further, although the gate would have accepted it (5 m after 100 m).
- **Scan Context** (`loop_sc_enabled`), only when no radius candidate passed. It compares
  gravity-levelled scan descriptors instead of positions, so it finds a revisit after more drift
  than the radius search reaches. Its yaw estimate seeds a coarse GICP (3× voxel, 3×
  correspondence distance) before the normal one.
- **Gates.** Over the overlap only (source points with a submap point within the 1 m
  correspondence distance): overlap ratio, inlier ratio, mean squared distance; optionally the
  corridor degeneracy ratio. Then the **drift gate**: the correction must be at most
  `loop_max_drift_ratio` of the path driven since the target. Look-alike places (symmetric
  corridors, the far side of a hall) align with a good inlier ratio but ask for a jump of their
  separation; Scan Context only considers targets this gate could pass.
- **Planar, robust loop edge.** A 6-DOF loop edge bent the chain and produced a double floor; the
  `PlanarLoopFactor` corrects only world x / y / yaw and leaves z / roll / pitch to the
  gravity-aligned odometry. It carries a Huber kernel, and the update iterates until the newest
  keyframe settles.
- **Logged.** Every candidate prints a `[PGONode][loop] candidate` line with its numbers and why
  it was rejected; every accepted loop prints two more lines with the correction and the
  keyframes' z before and after. Calibrate the gates from these on a replay.

`test/test_loop_closure.cpp` drives a synthetic, symmetric hall with drifting odometry through all
of this (`LOOP_DEBUG=1` prints every candidate against the ground truth).

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

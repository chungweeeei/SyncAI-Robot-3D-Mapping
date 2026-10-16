# Semantic mapping: design notes

Notes from a 2026-10-15 discussion on adding image semantics to the `pgo_node` map. Nothing here is
implemented; it records what was decided, why, and what is still missing. The pipeline today is
LiDAR-only: `pcl::PointXYZI` end to end, no camera subscription, no intrinsics or extrinsics
anywhere in the repo or in `SyncAI-Robot-Workspace`'s `description/G23.urdf`.

## Goal

Keep producing `map.pcd` / `octomap.bt` as now, and alongside them a map where each point (or
voxel) carries a semantic class from the camera: floor / stairs / grass / wall / door / person and
so on, for traversability and for "go to the X" style tasks. Closed-set labels first; CLIP-feature
(open-vocabulary) voxels are a later option.

## Decision: 2D segmentation + projection, attached to keyframes

A 2D image model gives the semantics, the LiDAR gives the geometry, Point-LIO gives the pose. No 3D
network is needed (CLIP2Scene-style distillation only pays off when semantics are wanted outside
the camera's field of view, and it needs a per-platform pre-training first). Every recent system
found (see references) is this same recipe.

```
/<robot_id>/pointlio/body_cloud_dense  (PointXYZI, body frame)
/<robot_id>/pointlio/lio_odom
<camera>/image_raw/compressed  +  K, D, T_cam_lidar
            |
   new node: semantic_labeler
   +- interpolate lio_odom to the image stamp
   +- project points into the image, z-buffer per pixel against occlusion
   +- label each point (class id, or a CLIP feature)
            |
   body_cloud_semantic (PointXYZRGBL or an extra field)
            |
   pgo_node unchanged in behaviour; patches/<i>.pcd now carry the label
            |
   after save_maps: fuse per voxel with the corrected poses.txt
   -> map_semantic.pcd  +  octomap.bt (occupancy only)  +  voxel -> class histogram
```

**Why attach semantics to keyframes, not to world-frame voxels.** `pgo_node` corrects keyframe
poses on loop closure. A semantic voxel grid accumulated in `map` during the run would be left
misaligned with the corrected cloud. Labelling the body-frame patch instead means the semantics
move with the keyframe, and `patches/ + poses.txt` is already the hand-off for exactly this kind
of post-save pass (`clean_map` works the same way). The 2026 LIO-plus-RGB meshing paper below
avoids loop closure altogether for this reason; we do not have to.

**Fusion rule.** Per voxel keep a class histogram (or a discrete Bayes filter over classes) and
take the argmax. Do **not** encode labels as pseudo-colours in a `ColorOcTree` and average them:
multi-frame averaging blends colours into classes that do not exist (the 2025 Semantic OctoMap
comparison reports exactly this).

## Touch points in this repo

- Point type is `pcl::PointXYZI` throughout (`include/syncai_mapping/pgos/commons.h`). Either
  switch to a labelled type or carry a parallel label array aligned to the point index.
- `clean_map` can use the label directly: points classed `person` are dropped outright instead
  of relying on the ray-cast miss ratio.
- `octomap.bt` stays pure occupancy. Semantics live in a separate `voxel -> histogram` table (or a
  second PCD with a `label` field), not inside the tree.
- The cross-repo contract (topics, services, `/dev/shm/syncai_pgo/<robot_id>`) stays as is; the
  labeller sits between `syncai_pointlio` and `pgo_node` and publishes a new topic.

## 2D model choice

| Model | Classes | Output | Good at | Notes |
|---|---|---|---|---|
| Mask2Former (ADE20K) | fixed (150) | per-pixel label | things and stuff | fast, stable, HF/Detectron2 weights; ADE20K already has floor, wall, stairs, grass, door, person |
| Grounded-SAM / SAM 2 | any text | instance masks | objects (things) | heavy; weak on amorphous stuff (floor, grass); best for object-level maps |
| OpenSeg / SAN / CAT-Seg / MaskCLIP | any text | per-pixel CLIP feature | open-vocabulary query after mapping | memory-hungry voxels; OpenSeg is TF-only |

Start with Mask2Former ADE20K for traversability classes; add Grounded-SAM later for object
instances; feature maps only if post-hoc text query becomes a requirement.

## Camera: what exists and what is missing

Camera is the TechNexion **VCS-AR0234-C**, 1280x720, driven two ways in the Workspace:

| Path | Publishes | `camera_info` |
|---|---|---|
| `vizionsdk_ros2` (`bringup.launch.py use_camera:=true`) | `image_raw/compressed` (MJPG), IMU | yes, only when `VxGetIntrinsics` succeeds (`vizionsdk_camera_node.cpp`, `LoadCameraInfo`); otherwise a warning and nothing |
| `syncai_camera_feeder` (GStreamer / WebRTC) | `image_raw/compressed` only | no |

Still needed before any projection:

1. **Intrinsics** `K`, `D`, image size. Check the bag with `ros2 bag info` for a `camera_info`
   topic. If present, validate it: the SDK values are the module's factory calibration, possibly at
   a different native resolution (the second unit advertises 1280x800), so a crop to 720 shifts
   `cy`. Undistort one frame and check straight lines, then overlay a projected scan. If absent or
   wrong, calibrate with a checkerboard:
   `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=<image topic>`.
2. **Extrinsics** `T_cam_lidar`. Not in the URDF, not in any params file. Calibrate with
   [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)
   (bag in, Mid-360 supported, needs the intrinsics first).
3. **Pose interpolation** to the image stamp. Camera and LiDAR stamps differ and a walking
   quadruped moves enough between them to misproject.
4. **Always check an overlay** (scan projected onto the image) before building anything on top.

Images are JPEG; `cv2.imdecode` is enough offline.

## Known pitfalls

- Occlusion / parallax: camera and LiDAR are at different positions, so points the camera cannot
  see get the foreground's label unless a per-pixel depth test keeps only the nearest point.
- Mask edges: projection error plus imprecise mask boundaries mislabel border points. Erode masks
  or down-weight points near a label boundary.
- Dynamic objects leave ghosts; let `person` voxels decay or never write them.
- Only points inside the camera frustum get a label; the rest stay unlabelled until a 360 camera
  or a 3D network (CLIP2Scene / LOSC) fills them in.

## Plan

1. **Offline first, no ROS changes.** Bag -> Point-LIO + PGO -> `save_maps` with
   `save_patches` -> for each keyframe pick the nearest image -> Mask2Former -> project labels
   onto the patch -> compose with `poses.txt` -> per-voxel majority vote -> coloured semantic
   `.ply` for CloudCompare / RViz. Pure Python; validates the whole idea fastest.
2. Promote the labeller to a ROS 2 node between Point-LIO and `pgo_node`; change the point type.
3. Only then consider Grounded-SAM for object instances or CLIP2Scene for out-of-view semantics.

## First offline result (2026-10-16, `dp1f_with_image`)

Plan step 1, run end to end on `record/dp1f_with_image` (784 s, Mid360 + IMU, camera at 30 Hz,
`camera_info` from the chessboard calibration, `tf_static` carrying the provisional hand-measured
mount from `G23.urdf`). Scripts and the exact commands: `tools/semantic_offline/README.md`.
Outputs: `map/dp1f_with_image/semantic/` (`map_semantic.{pcd,ply}`, the `_clean` variant masked
by the cleaned `map.pcd`, `*_stats.json`, top-down views, `qa/` overlays).

| Step | What was done | Result |
|---|---|---|
| Replay | pointlio + `pgo_node` (`start_on_launch`) in the robot image, `save_maps` with patches, `clean_map` | 1071 keyframes, 10 loops, 8.2 M points; clean `ok` in 51 s |
| Keyframe stamps | `poses.txt` has none. Recorded `lio_odom` and pgo's `map → pointlio_odom` TF (one per processed pair) during the replay and re-ran `isKeyPose` over the processed pairs | 1071 = 1071, worst consecutive relative-motion mismatch 1 mm, all 637 `loop_markers` stamps found |
| Extrinsic check | `overlay_test.py`: raw scans projected onto the nearest image at 4 moments | floor / wall boundary agrees to a few px at 2–3 m; the hand-measured mount is usable for stuff classes, not yet for thin objects |
| 2D model | Mask2Former Swin-L ADE20K (`transformers`), 512 px short side | 0.33 s / image on the Mac's MPS (Swin-T: 0.21 s); floor / wall / ceiling / door / person clean on the test frames |
| Projection | nearest image per keyframe (mean \|dt\| 7 ms, max 35 ms), body pose interpolated through `lio_odom` at the image stamp, 8 px z-buffer against occlusion, weight × 0.3 on label boundaries | 16 % of each patch's points lie in the frustum; 1071 keyframes in 326 s |
| Fusion | 0.1 m voxels, weighted class histogram, argmax | 1.34 M voxels, 31 % labelled (40 % of the 0.90 M voxels left after masking with the cleaned `map.pcd`), mean vote purity 0.95 |

Class shares of the labelled voxels: floor 36 %, wall 35 %, ceiling 18 %, then windowpane,
door, column, shelf, armchair, chair, sofa, counter, light; `person` 0.3 % (1457 voxels, the
ghosts `clean_map` removes from `map.pcd`; the masked variant inherits that removal, the raw
fusion does not); `stairway` 801, `escalator` 990.

What it says about the plan:

- The recipe works as designed and nothing in `pgo_node` had to change. The one thing the save
  lacks is the keyframe stamp: a `stamps.txt` beside `poses.txt` (or a column, if the backend's
  reader is checked first) would replace the replay-time recording.
- Grounded-SAM 2 was not run: closed-set stuff classes were the first target, and Mask2Former
  covers them. The labeller takes any `Segmenter` with the same `(label, confidence, boundary)`
  output, so an object-instance pass is a backend swap.
- The extrinsic is the limit before the model is: calibrate (`direct_visual_lidar_calibration`)
  before judging object classes, and keep the overlay check in the loop.
- Only the forward frustum is labelled, so two thirds of the map stays unlabelled; a wider lens
  or a second camera matters more than a better model for coverage.
- Fusion is per patch at keyframe density (0.5 m / 10°); labelling every scan between keyframes
  would multiply the votes per voxel, at 10× the segmentation cost.

## References

- [gfchen01/semantic_mapping_with_360_camera_and_3d_lidar](https://github.com/gfchen01/semantic_mapping_with_360_camera_and_3d_lidar):
  ROS 2 node, images + 3D LiDAR + odometry, Grounded-SAM-2, object-level map, sample bag. Closest
  existing system.
- [A Comparison of Segmentation Methods for Semantic OctoMap Generation (2025)](https://www.doi.org/10.3390/app15137285):
  ROS 2 labelling into `octomap_server`; documents the pseudo-colour blending problem.
- [Camera-Lidar Integration: Probabilistic sensor fusion for semantic mapping (2020)](https://arxiv.org/pdf/2007.05490):
  discrete Bayes filter per voxel over classes.
- [Fusing Lidar and Semantic Image Information in Octree Maps (ACRA 2017)](https://www.araa.asn.au/acra/acra2017/papers/pap109s1-file1.pdf):
  projection + Octomap, down-weights points near mask boundaries.
- [Incremental Semantics-Aided Meshing from LiDAR-Inertial Odometry and RGB Direct Label Transfer (2026)](https://arxiv.org/pdf/2604.09478):
  FAST-LIO2 backbone, per-voxel label histogram, no loop closure.
- [FindAnything (2025)](https://arxiv.org/pdf/2504.08603), [OVI-MAP (2026)](https://arxiv.org/pdf/2603.26541),
  [LOSC (2025)](https://arxiv.org/pdf/2507.07605): open-vocabulary object maps and LiDAR
  open-vocabulary segmentation; research code, ROS 2 support unverified.
- [CLIP2Scene (CVPR 2023)](https://arxiv.org/pdf/2301.04926.pdf): 2D-to-3D CLIP distillation;
  inference needs only the point cloud, but needs re-pre-training on our LiDAR and scenes.
- [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration):
  LiDAR-camera extrinsic calibration from a bag.

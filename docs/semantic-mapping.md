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
instances; feature maps only if post-hoc text query becomes a requirement. Seven backends were
then run through the same projection and fusion on 2026-10-17 — see "2D backend comparison"
below for what actually separates them on our data.

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

## 2D backend comparison (2026-10-17, `dp1f_with_image`)

The same replay (1066 keyframes this time — a real-time replay is not bit-for-bit repeatable —
13 loops, 8.2 M points, `clean_map` ok in 84 s), the same keyframe stamps, projection and fusion,
and only the 2D model swapped (`tools/semantic_offline/segment.py`, one class per backend). The
runs live in `map/dp1f_with_image/semantic/<tag>/`, the table and a page with every run's
top-down renders and QA overlays side by side in `semantic/compare/compare.{md,html}` (raw
fusion) and `semantic/compare_clean/` (masked by the cleaned `map.pcd`).

There is no annotated ground truth, so `compare.py` scores against two things: the map's own
geometry (height over the local floor from the nearest keyframe minus the 0.481 m lidar height,
and the voxel normal), and the Swin-L reference run, after folding every label space into 16
coarse classes (`taxonomy.py`). "floor low+flat" is the share of floor-labelled voxels within
0.25 m of the floor with a near-vertical normal; "geo-floor → floor" is the share of labelled
voxels *with* floor geometry that got the floor label. The ceiling rate is weak on purpose
(ducts, beams and the mezzanine count against it) and is only comparable across rows. Times
are seconds per 1280×720 image on this Mac's MPS, including the upsampling back to full
resolution and the boundary mask; use them as ratios.

| run | s / image | labelled | purity | floor low+flat | geo-floor → floor | wall vertical | agree w/ Swin-L |
|---|---|---|---|---|---|---|---|
| Mask2Former Swin-L ADE20K (reference) | 0.96 | 31 % | 0.95 | 80 % | 98 % | 82 % | – |
| Mask2Former Swin-T ADE20K | 0.53 | 31 % | 0.95 | 81 % | 97 % | 80 % | 88 % |
| SegFormer-B5 ADE20K 640 | 0.62 | 32 % | 0.96 | 80 % | 98 % | 82 % | 89 % |
| SegFormer-B2 ADE20K 512 | 0.40 | 32 % | 0.96 | 79 % | 98 % | 81 % | 87 % |
| OneFormer Swin-L ADE20K | 1.27 | 32 % | 0.95 | 81 % | 97 % | 81 % | 92 % |
| Mask2Former Swin-L COCO panoptic | 1.29 | 32 % | 0.93 | 81 % | 97 % | 80 % | 83 % |
| Grounding DINO tiny + SAM ViT-B (18 prompts, every 3rd keyframe) | 3.45 | 8 % | 0.97 | 84 % | 99 % | 96 % | 90 % |

Coarse class shares of the labelled voxels, raw fusion (%):

| run | floor | wall | ceiling | door | person | seat | table | column | window | storage | other |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Swin-L ADE | 36.5 | 34.7 | 18.3 | 0.9 | 0.4 | 2.0 | 0.8 | 0.9 | 2.2 | 1.2 | 0.9 |
| Swin-T ADE | 36.1 | 37.9 | 17.5 | 0.8 | 0.3 | 1.3 | 0.4 | 1.5 | 2.2 | 0.2 | 0.5 |
| SegFormer-B5 | 36.7 | 36.1 | 20.4 | 0.6 | 0.4 | 1.4 | 0.3 | 0.6 | 2.3 | 0.2 | 0.3 |
| SegFormer-B2 | 37.0 | 36.5 | 21.2 | 0.5 | 0.4 | 1.2 | 0.1 | 0.3 | 1.7 | 0.1 | 0.3 |
| OneFormer Swin-L | 35.9 | 35.1 | 17.3 | 1.2 | 0.3 | 2.2 | 1.6 | 1.3 | 1.9 | 0.4 | 0.9 |
| Swin-L COCO | 35.3 | 30.2 | 16.0 | 1.7 | 0.4 | 3.5 | 0.7 | 0.0 | 5.4 | 0.8 | 3.7 |
| Grounded-SAM | 75.0 | 6.3 | 6.3 | 1.7 | 0.9 | 3.1 | 0.5 | 1.5 | 1.6 | 0.9 | 0.0 |

What the numbers say:

- **On the traversability classes the model does not matter.** Every ADE20K model lands within
  two points of the others on floor / wall / ceiling, with the same coverage (16 % of each patch,
  31 % of the voxels — the frustum, not the model, sets it) and the same purity. The floor layer
  of the map (`view_topdown_lowband*.jpg`) is interchangeable between them.
- **The models differ on objects and on mask edges.** Swin-L and OneFormer find roughly twice the
  seat / table / storage / column voxels that SegFormer-B2 does (2.0 / 0.8 / 1.2 / 0.9 % against
  1.2 / 0.1 / 0.1 / 0.3 %), and B2's masks are blobbier: in the QA overlays the floor bleeds up
  around people's feet and the ceiling / wall boundary wanders. The voxel disagreement with the
  reference (8–13 % for the ADE models) sits almost entirely there; the reference-confusion in
  `compare.json` shows floor agreeing at 97–99 %, wall at 86–92 %, ceiling at 88–95 %.
- **SegFormer-B2 is the on-robot candidate.** 2.4× faster than Swin-L here (and a 27 M-parameter
  model that TensorRT handles on a Jetson), for a floor / wall / ceiling layer that is the same.
  Re-run the large model offline after a save, from the patches, when object labels matter.
- **COCO panoptic buys nothing on this site.** It reads the glass façade as `window` (5.4 %),
  puts 3.7 % into classes outside the taxonomy, and agrees least with the reference (83 %); its
  wall precision against the reference is 77 %. ADE20K's 150 classes already cover what the map
  needs (floor, wall, stairs, escalator, door, person, column).
- **Grounded-SAM is an object pass, not a semantic model.** Boxes are the wrong primitive for
  stuff: only 8 % of the voxels get a label and 75 % of those are floor; walls and ceilings go
  almost unlabelled (the few it does label are right, hence the 96 % wall rate). It does find the
  seats, doors, stairs and people (3.1 / 1.7 / 1.0 / 0.9 %) with the cleanest per-object masks of
  the seven, at 3.5 s per image — ~10× the closed-set models. If object instances are wanted, run
  it as a second pass on top of an ADE20K stuff map, not instead of one; SAM 3 and the single-pass
  EOVSAM (references) are the versions to try when that happens.
- **The pipeline's own limits still dominate the model's.** Coverage (31 % of voxels labelled),
  the hand-measured extrinsic, and the ~1500 `person` ghost voxels every model carries into the raw
  fusion (the masked variant halves them, because `clean_map` has already removed the points)
  move the result more than any backend swap. Model choice is settled for now: next is coverage
  and calibration, then the keyframe stamp sidecar so the replay recording goes away.

Two things the run turned up about the tooling: `poses.txt` still has no stamps, so the replay
recording is still needed, and a workspace whose `build/` has been deleted leaves
`install/syncai_common` as dangling `--symlink-install` links — the C++ nodes keep running, but
`ros2 service call` cannot import the service type (`replay.sh` now checks before playing;
rebuilding the package into a scratch prefix is enough).

## Object map (2026-10-18, `dp1f_with_image`)

The semantic voxels answer "what is here"; "where is the green sofa" needs objects. Two ways to
get them were tried on the same replay, both ending in the same `objects.json` (class, colour
word, up to three crops, a SigLIP 2 embedding of the crops, a free floor pose 0.6–1.5 m away
facing the object) and the same text query (`query_objects.py`, Chinese or English, cosine over
the object embeddings). Outputs: `semantic/objects/` and `semantic/instances_grounded-sam/`.

| | clustering (`objects.py`) | masks + association (`instances.py`) |
|---|---|---|
| source | DBSCAN on the Swin-L map's "thing" voxels | Grounding DINO + SAM masks every 2nd keyframe, projected, associated |
| time | ~2 min on top of the semantic run | 18 min segmentation (2.0 s / image), association seconds from the cache |
| objects | 114 | 188 |
| teal lounge: sofa / armchair boxes (~18–21 pieces in the map) | 26, several sofas boxed twice | 20, boxes thinner (only the faces seen), a few sofas missed |
| crops | low-angle, often half a sofa | one whole object per crop (the mask's box) |
| false positives | few (ADE classes are reliable on furniture) | many: "escalator" on corridors, glass doors and the staircase, "armchair" on bar stools and dining chairs |
| small things | none (below the voxel threshold) | fire extinguisher, trash can, ATM-like machines found |

Both answer "綠色沙發" / "green sofa" with the dark teal sofas by the façade, and "grey leather
armchair" with the grey-olive ones in the north lounge. The colour word is unreliable (teal reads
`cyan` or `black`, grey-olive reads `green`; the white balance is latched), the embedding is not.
Rank by cosine: with the prompt ensemble SigLIP's sigmoid is near 1 for every object of the right
class, so "not found" needs its own rule (margin to the second hit, or class + embedding).

Association lessons, each from a failed run: shared 0.2 m voxels miss (two views hit different
faces of a sofa); an unbounded footprint rule chains a lounge into 5–7 m "sofas"; nearness has to
be measured in the floor plane; and a mask's ~20 LiDAR points include a few that went through a
gap onto something behind (robust footprint 1.4 m, full 2.4 m, p90 4.4 m), so each segment is cut
to its largest 3D cluster first. After that the footprints are plausible (median 1.1 m).

What limited it was the detector, not the association: Grounding DINO's labels are the false
positives above. **SAM 3** (`--backend sam3`, run on 2026-10-18 once access was granted) fixes
that, on the same 533 images, prompts and association (`semantic/instances_sam3/`):

| | Grounding DINO + SAM | SAM 3 | SAM 3, `--min-obs 4` |
|---|---|---|---|
| s / image (MPS, 18 prompts) | 2.0 | 10.9 | (same segments) |
| segments / objects kept | 985 / 188 | 1332 / 189 | 1332 / 101 |
| "escalator" objects | 21, nearly all corridors / glass doors | 1 (the staircase from another side) | 0 |
| teal lounge sofa + chair boxes (~18–21 pieces) | 20 | 33 | 21 |
| small objects (extinguisher, trash can, plants, ATM) | some, mixed with false hits | all found, crops correct | most (one ATM-like machine, one plant lost) |

With SAM 3 every crop in the gallery is the thing its label says, the ATM is labelled
`vending machine` (the closest prompt), and the staircase is `stairs`. Objects are seen far more
often (sofas typically 20–34 observations, against 2–10), which is what makes `--min-obs` a usable
filter: the extra lounge boxes at the default 2 are thin 2–3-observation slivers of sofas already
found. The queries ("綠色沙發", "grey leather armchair", "fire extinguisher", "trash can", "stairs")
return the right objects. The cost is time: one forward pass per prompt, 5.4× Grounding DINO;
offline that is ~95 minutes for this bag. **SAM 3 is the instance backend; `--min-obs 3–4` the
setting for it**, and trimming the prompt list to what a site actually needs is the speed knob.

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
- [SAM 3: Segment Anything with Concepts](https://github.com/facebookresearch/sam3) and
  [EOVSAM (2026)](https://www.alphaxiv.org/abs/2608.02284): text-prompted instance segmentation
  and its single-pass semantic variant — the open-vocabulary backend to try after Grounded-SAM
  (one pass per concept is what makes the latter ~10× slower than the closed-set models).
- [FUS3DMaps (2026)](https://arxiv.org/pdf/2605.03669), [OpenVox (2025)](https://arxiv.org/html/2502.16528v1),
  [SLAMVDB](https://robogroup.edi.lv/technology/slamvdb): open-vocabulary voxel maps with a
  dense and an instance layer (the last one ROS 2 packages with vision-language embeddings per
  voxel) — where this pipeline would go if text query after mapping becomes a requirement.

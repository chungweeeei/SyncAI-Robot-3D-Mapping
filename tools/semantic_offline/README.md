# Offline semantic map (step 1 of `docs/semantic-mapping.md`)

Pure-Python, no ROS changes: a replayed bag's `patches/ + poses.txt` are labelled from the camera
images and fused into a semantic voxel map, once per 2D backend, and the runs are compared on the
map's own geometry. Results and what they say: `docs/semantic-mapping.md`, "First offline result"
and "2D backend comparison".

```
bag ──replay/replay.sh (robot image)──▶ map/<name>/{patches,poses.txt}  +  replay/{lio_odom,tf_map_odom}.txt
                                                   │
keyframes.py      stamps per keyframe (poses.txt has none): re-run isKeyPose over the pairs pgo processed
extract_images.py (compressed bag only) the image nearest each keyframe + camera_info + tf_static → a small bag
label_patches.py  nearest image → 2D backend → pose-interpolated projection → <run>/labels/<i>.npz
fuse.py           patches × poses.txt → 0.1 m voxels → weighted class histogram → argmax → map_semantic.{pcd,ply}
render.py         top-down rasters + legend per run
compare.py        runs side by side: coarse taxonomy, geometry-based rates, agreement, HTML page
```

## Run

1. **Replay** the bag through pointlio + pgo_node in the Workspace's robot image and save with
   patches; `replay/replay.sh` is the script, `replay/recorder.py` records what the stamp
   recovery needs (`/robot01/pointlio/lio_odom`, and pgo's `map → pointlio_odom` TF, one per
   processed pair). Run it on an isolated network: it does not need the bag's own `/tf`.
   ```
   docker run -d --shm-size=2g --user $(id -u):$(id -g) \
     -v <workspace>:/home/syncrobotic/robot_ws -v <dir with replay.sh + recorder.py>:/scratch \
     -e HOME=/home/syncrobotic -e ROS_DOMAIN_ID=77 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
     -e CYCLONEDDS_URI=/home/syncrobotic/robot_ws/config/cyclonedds.xml \
     syncai-robot-base bash /scratch/replay.sh
   ```
   `replay.sh [<bag> [<map dir>]]` (container paths; defaults `record/dp1f_with_image_db3` →
   `map/dp1f_with_image`) plays in real time, saves with patches, waits for `clean_map` and runs
   `clean_map --octomap 1` when the save left no `octomap.bt`. The bag must be an **uncompressed
   view**: a directory with the recording's `metadata.yaml` rewritten to `compression_format: ""`,
   `compression_mode: ""` and `.db3` names, so play never inflates into the container. For a
   recording too large to inflate whole (`dp1f_1022_with_image`: 7 × 2 GB on a disk with 16 GB
   free) build the view in the scratch directory with files 0 and 1 inflated, then keep it fed
   from the host while the container runs:
   ```
   replay/feed_bag.sh <container> <view dir> <recording dir> <bag base name>
   ```
   It reads which `.db3` the player holds open off the container's `/proc/*/fd`, inflates the
   next file (~2 s) and deletes the previous one, so at most two files exist at a time.
2. **Label, fuse and compare**, on the host:
   ```
   uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python -r requirements.txt
   ./run_all.sh <bag dir> <map dir> <replay output dir> <out root> [model[:stride] ...]
   .venv/bin/python compare.py <map dir> <out root>/compare <out root>/<tag>...
   ```
   Without a model list `run_all.sh` runs the seven backends below into `<out root>/<tag>/`;
   `:N` labels every N-th keyframe. `bagio.py` finds the Livox `.msg` files in the Workspace
   checkout this package sits in (`LIVOX_MSG_DIR` overrides).

   `<bag dir>` is read by `rosbags`, which inflates every file of a zstd file-compressed
   recording at open. For such a recording make the image-only bag first and pass that instead:
   ```
   .venv/bin/python extract_images.py <recording dir> <image bag dir> --keyframes <out root>/keyframes.txt
   ```
   One file at a time (inflate, read, delete), it keeps the image nearest each keyframe stamp
   plus `camera_info` and `tf_static`, with the original stamps and receive times, so
   `ImageIndex` and `read_static` see what they would see in the full bag (0.5 GB for 2 000
   keyframes instead of 14 GB). `--every-s N` keeps one image every N seconds on top.

`overlay_test.py <bag> <out> [seconds...]` projects raw scans onto images: run it first on any new
extrinsic. `seg_test.py <bag> <out> <model>...` times and overlays any backend on four bag images.

The projection follows the bag's `camera_info`: `plumb_bob` through `cv2.projectPoints`,
`equidistant` (robot01's `camera0.yaml` since the 2026-10-20 calibration, 110° × 60° on the raw
stream) through `cv2.fisheye.projectPoints` — a 4-coefficient equidistant `D` fed to the plumb_bob
call reads as `k1 k2 p1 p2` and bends every point near the border the wrong way. The pre-projection
cone (`bagio.view_cone`) is per axis for plumb_bob (the values the 2026-10-16/17 runs used) and an
off-axis angle derived from `K` and the image corners for equidistant, because the fisheye sees
~61° off axis at the corners and `|x/z| < 1.1` (48°) would drop the outer third of the image.

## 2D backends (`segment.py`, `make_segmenter(name)`)

| name | backend | label space |
|---|---|---|
| `facebook/mask2former-swin-large-ade-semantic` (reference), `-swin-tiny-` | Mask2Former | ADE20K 150 |
| `nvidia/segformer-b5-finetuned-ade-640-640`, `-b2-…-512-512` | SegFormer (the Jetson-sized ones) | ADE20K 150 |
| `shi-labs/oneformer_ade20k_swin_large` | OneFormer | ADE20K 150 |
| `facebook/mask2former-swin-large-coco-panoptic` | Mask2Former, semantic map from a panoptic head | COCO panoptic 133 |
| `grounded-sam` | Grounding DINO tiny + SAM ViT-B, prompts in `GROUNDED_PROMPTS` | the prompt list; pixels no box covers stay unlabelled |

All return the same `(label uint8, confidence, boundary)` per pixel. `taxonomy.py` folds every
label space into 16 coarse classes (floor, wall, ceiling, door, person, stairs, seat, table,
column, window, storage, plant, sign, light, escalator, other) for `compare.py` and the coarse
top-down renders; the fused maps keep the fine labels.

## Object map (`objects.py`, `query_objects.py`)

Clusters the "thing" voxels of a fused map into instances (DBSCAN per coarse group, so a sofa
voted half `sofa` half `armchair` stays one object), picks each one's best views from the
keyframe images (frustum, patch z-buffer, 1.2 m minimum distance, not cut by the border), and
stores per object a class, a colour word from the pixels its own voxels land on, up to three
crops, a SigLIP 2 embedding (mean of the crops) and a free floor pose 0.6–1.5 m away facing it.

```
.venv/bin/python objects.py <map dir> <run dir> <bag dir> <keyframes.txt> <lio_odom.txt> <out dir>
.venv/bin/python query_objects.py <out dir> "green sofa" "綠色沙發" ...
```

Writes `objects.json` (no map name, no absolute path), `objects_emb.npy`, `crops/`,
`index.html` (every object), `objects_topdown.jpg` (ids on the floor plan) and
`query.{html,json}`. SigLIP 2 is multilingual, so Chinese queries need no translation. Rank
by cosine: with the prompt ensemble the sigmoid `p` is no longer calibrated, and every object of
the right class scores close to 1.

## Instance-level object map (`instances.py`, `instance_seg.py`)

The same `objects.json` as `objects.py`, but objects come from 2D instance masks instead of
distance clustering of labelled voxels. Phase 1 (slow, cached in `segments_<backend>.pkl`):
text-prompted instance masks on every `--stride`-th keyframe image, each mask's visible patch
points in the world frame. Phase 2 (seconds, `--reuse`): each segment is cut to its largest 3D
cluster, then joins the object of its coarse group that has `--match` of its points within
`--near` in the floor plane, or whose grown footprint holds its centroid, never past
`--max-extent`; objects that end up near each other merge. A `dedup` pass then gives each
physical object one id (cross-class overlaps and same-class fragments merge, unless a frame saw
them as separate masks), and `--min-obs` drops one-frame hits; use `--min-obs 4` with SAM 3.

```
.venv/bin/python instances.py <map dir> <bag dir> <keyframes.txt> <lio_odom.txt> <out dir> [--backend sam3|grounded-sam]
.venv/bin/python instances.py ... --reuse --near 0.3 --max-extent 3.5     # retune the association only
```

`--backend sam3` needs the gated `facebook/sam3` checkpoint: request access on its Hugging Face
page, then `hf auth login`. The backend follows the transformers 5.20 API (shared vision
features, one forward per prompt) but has not been run against the weights yet.

## Semantic octomap viewer (`semantic_octomap_viewer.py`, `octomap_io.py`)

One self-contained HTML page (three.js, ~10 MB) with the octomap's occupied cells coloured by
class and the object map on top. Needs `octomap.bt` (`clean_map <map dir> --octomap 1`, which
reproduces `map.pcd` exactly from `patches/`).

```
.venv/bin/python semantic_octomap_viewer.py <map dir> <semantic run dir> <objects dir> <out.html>
```

- `octomap_io.read_bt` reads the binary tree without the octomap library; an octomap cell of a
  0.1 m tree is a fuse.py voxel key, so the semantic voxels index it directly.
- Objects (`objects_voxels.npz` from `instances.py`) are painted onto the cells they cover:
  seeds within 0.25 m of a mask's points, region-grown through occupied cells above the walked
  floor, bounded by the class size and height. Box, centroid and tag come from the painted cells.
- Tags are sprites in the scene (depth-tested, constant screen size) whose tip sits on one of
  the object's own cells; an HTML overlay floated in front of everything.
- Modes: fused (object class over voxel semantics), voxel semantics, object instances, height.
  `#obj=<id>[&az=<deg>]` in the URL flies to an object.

## What the files hold

- `<run>/labels/<i>.npz`: `label` (uint8 per patch point, the backend's id, 255 = outside the
  camera view or occluded), `weight` (model confidence, ×0.3 on a label boundary), the image
  index and |dt|. `<run>/run.json`: model, device, s / image, keyframes, stride.
- `<run>/map_semantic.pcd`: voxel centroids with `rgb` (class colour), `label` (uint32, 255
  unlabelled) and `confidence` (winning share of the voxel's votes); `.ply` is the same for
  CloudCompare. `map_semantic_clean.*` keeps only voxels that still hold a point of the cleaned
  `map.pcd`. `*_stats.json`: voxel counts per class; `classes.json`: id → name and the palette;
  `view_topdown_*.jpg`, `coarse_topdown*.jpg`, `legend*.png`, `qa/kf_*.jpg` (overlays every 100th
  keyframe, same indices in every run).
- `compare/compare.{json,md,html}`: the table and the page with every run's renders and QA
  overlays side by side.

## Known limits

- Bags before 2026-10-20 carry the URDF's hand-measured extrinsic and a plumb_bob fit of the
  stabilized stream; `dp1f_1022_with_image` and later carry the calibrated equidistant intrinsics
  and the fitted extrinsic (`docs/lidar-camera-calibration.md` in the Workspace). Check the
  overlay on any new bag either way.
- Only the forward frustum gets labels (~16 % of each patch); the rest is grey.
- `person` voxels are ghosts from the raw patches; `clean_map` removes them from `map.pcd`, the
  masked variant inherits that, the raw fusion does not.
- `compare.py`'s rates use the map's geometry as the only ground truth, so they cover floor /
  wall / ceiling and nothing else; the QA overlays are the check for objects.

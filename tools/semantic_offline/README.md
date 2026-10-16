# Offline semantic map (step 1 of `docs/semantic-mapping.md`)

Pure-Python, no ROS changes: a replayed bag's `patches/ + poses.txt` are labelled from the camera
images and fused into a semantic voxel map. First run and its numbers: `docs/semantic-mapping.md`,
"First offline result".

```
bag ──replay/replay.sh (robot image)──▶ map/<name>/{patches,poses.txt}  +  replay/{lio_odom,tf_map_odom}.txt
                                                   │
keyframes.py      stamps per keyframe (poses.txt has none): re-run isKeyPose over the pairs pgo processed
label_patches.py  nearest image → Mask2Former (ADE20K) → pose-interpolated projection → labels/<i>.npz
fuse.py           patches × poses.txt → 0.1 m voxels → weighted class histogram → argmax → map_semantic.{pcd,ply}
render.py         top-down rasters + legend for a quick look
```

## Run

1. **Replay** the bag through pointlio + pgo_node in the Workspace's robot image and save with
   patches; `replay/replay.sh` is the script, `replay/recorder.py` records what the stamp
   recovery needs (`/robot01/pointlio/lio_odom`, and pgo's `map → pointlio_odom` TF, one per
   processed pair). Run it on an isolated network: it does not need the bag's own `/tf`.
   ```
   docker run -d --shm-size=2g --user $(id -u):$(id -g) \
     -v <workspace>:/home/syncrobotic/robot_ws -v <this dir>/replay:/scratch \
     -e HOME=/home/syncrobotic -e ROS_DOMAIN_ID=77 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
     -e CYCLONEDDS_URI=/home/syncrobotic/robot_ws/config/cyclonedds.xml \
     syncai-robot-base bash /scratch/replay.sh
   ```
   `replay.sh` expects the bag at `record/dp1f_with_image_db3` (a directory of `.db3` symlinks
   plus a `metadata.yaml` with `compression_format: ""`, so play does not inflate the zstd files
   into the container) and writes to `map/dp1f_with_image`; edit the two paths for another bag.
2. **Label and fuse**, on the host:
   ```
   uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python -r requirements.txt
   ./run_all.sh <bag dir> <map dir> <replay output dir> <out dir> [HF model]
   .venv/bin/python fuse.py <map dir> <out dir> <out dir> --mask-with <map dir>/map.pcd --name map_semantic_clean
   .venv/bin/python render.py <out dir>
   ```
   `bagio.py` needs the Livox `.msg` files (`LIVOX_MSG_DIR`, default: the Workspace checkout).

`overlay_test.py <bag> <out> [seconds...]` projects raw scans onto images: run it first on any new
extrinsic. `seg_test.py` times the two Mask2Former sizes on four bag images.

## What the files hold

- `labels/<i>.npz`: `label` (uint8 per patch point, ADE20K id, 255 = outside the camera view or
  occluded), `weight` (model confidence, ×0.3 on a label boundary), the image index and |dt|.
- `map_semantic.pcd`: voxel centroids with `rgb` (class colour), `label` (uint32, 255 unlabelled)
  and `confidence` (winning share of the voxel's votes); `.ply` is the same for CloudCompare.
  `map_semantic_clean.*` keeps only voxels that still hold a point of the cleaned `map.pcd`.
- `*_stats.json`: voxel counts per class; `classes.json`: id → name and the palette.

## Known limits

- The camera extrinsic is the URDF's hand-measured one; calibrate before trusting thin objects.
- Only the forward frustum gets labels (~16 % of each patch); the rest is grey.
- `person` voxels are ghosts from the raw patches; `clean_map` removes them from `map.pcd`, the
  masked variant inherits that, the raw fusion does not.
- Segmentation backend is one class: swap `Segmenter` in `segment.py` for Grounded-SAM 2 or a
  CLIP-feature model without touching the projection or fusion.

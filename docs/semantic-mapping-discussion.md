# Semantic mapping: discussion record (2026-10-17 / 18)

Questions asked while the offline semantic-map pipeline was being built, and the answers that
came out of the experiments. The measurements behind every claim here are in
[`semantic-mapping.md`](semantic-mapping.md) ("2D backend comparison", "Object map"); the tools
and their commands in `tools/semantic_offline/README.md`. This file records *why* the work went
the way it did, so the next decision starts from it.

## 1. Can camera images turn the LiDAR map into a semantic map?

Yes, without touching `pgo_node`. A 2D model labels each keyframe's nearest image, the labels
are projected onto the keyframe's patch (pose interpolated to the image stamp, z-buffer against
occlusion), and the patches are fused per 0.1 m voxel with the loop-corrected `poses.txt`
(class histogram, argmax). Run end to end on `record/dp1f_with_image`: 31 % of the voxels get a
label, vote purity 0.95. Attaching labels to keyframes rather than to world voxels is what keeps
them aligned after loop closure.

What it cost that the save does not provide: `poses.txt` has no stamps, so the replay has to
record `lio_odom` and pgo's `map → pointlio_odom` TF to recover them. A `stamps.txt` beside
`poses.txt` would remove that step.

## 2. What is the semantic map for?

Ranked by value over effort, with what each needs:

1. **Keepout masks generated from the map.** Stairs, escalators and glass façades: the Mid360
   sees through glass, so the LiDAR map calls a glass wall free space, while the camera labels it
   `window`. Projected to 2D and written as `keepout.yaml` / `.pgm`, it reaches the planner through
   the existing `filter_mask_server/load_map` path. Offline, no runtime change, closes a safety
   gap. **Recommended first.**
2. **The gridmap recipe choice.** The backend has two pcd → gridmap recipes and deliberately no
   automatic pick between them, for want of a reliable signal. A semantic floor layer is that
   signal.
3. **Pre-filled octomap floor edits.** The console's ground editor marks floor by hand before
   `apply_octomap_edits`; glossy floors leave holes and reflections. Semantic `floor` could fill
   `octomap_edits.json` and leave the operator to correct it.
4. **`clean_map` dropping `person` points outright** instead of relying on the miss ratio alone.
5. **Semantic goals** ("go to the door", "go to the green sofa"): needs objects, not voxels. See 4.
6. **Online uses** (slow down near people, keep dynamic points out of the localizer's
   registration, charger detection for the planned charger-dock tree): need a model on the
   Jetson; the most expensive.

## 3. With this many 2D models, which one?

Pick by where it runs and which classes matter, not by a leaderboard. Seven backends were run
through the identical projection and fusion; on floor / wall / ceiling every ADE20K model lands
within two points of the others, so for traversability the choice does not matter. They differ
on objects and mask edges.

| Use | Choice | Why |
|---|---|---|
| Offline, after a save | Mask2Former Swin-L ADE20K | Same floor layer, about twice the object voxels of the small models; latency is irrelevant offline |
| Online on the Jetson | SegFormer-B2 ADE20K | 2.4× faster here, same floor layer; weaker objects, blurrier edges. Jetson / TensorRT latency not measured yet |
| Objects, free-text names | SAM 3 (Grounding DINO + SAM until it is available) | A second pass on top of an ADE stuff map, never instead of one |
| Dropped | COCO panoptic, OneFormer | COCO misreads the glass façade and agrees least; OneFormer is 30 % slower than Swin-L for no visible gain |

Two caveats that bound this answer. Object classes have no real score yet: the comparison uses
the map's own geometry as ground truth, which only covers floor / wall / ceiling; comparing doors,
glass and stairs needs ~50 hand-labelled frames, worth doing only when item 1 or 6 of section 2
is started. And the pipeline's own limits move the result more than any model swap: only the
forward frustum is labelled (31 % of voxels) and the camera extrinsic is the URDF's hand-measured
one. Coverage and calibration come before model tuning.

## 4. Marking objects: "where is the green sofa?"

"Green sofa" is a class (ADE has `sofa`) plus an attribute (colour, which the voxel map does not
store), and "where" needs the voxels grouped into objects. Plan, in three stages:

1. **Instances by clustering** the "thing" voxels of the existing semantic map, per coarse group.
2. **Per-object attributes**: the best two or three views of each object (frustum, z-buffer,
   distance, not cut by the border), a crop of each, a colour word from the pixels the object's
   own voxels land on, and one SigLIP 2 embedding (mean of the crops). A text query is a cosine
   over the object embeddings; SigLIP 2 is multilingual, so "綠色沙發" works as typed. Per-object
   crops instead of per-pixel CLIP features: a query only ranks objects, and a few hundred image
   encodes per map is cheap.
3. **Instances from 2D masks** (SAM 3), associated across keyframes in the corrected world
   frame, to replace the clustering.

Every object also gets a free floor pose 0.6–1.5 m from it, facing it, so a hit can become a
MOVE goal. The output, `objects.json`, follows the map-directory contract: no map name, no
absolute path.

What the runs showed:

- Stages 1 + 2 answer "green sofa" with the dark teal sofas by the façade, and "grey leather
  armchair" with the grey-olive ones in the north lounge, in about two minutes on top of the
  semantic run. Clustering fragments: 42 sofa / armchair instances for a few lounges.
- The colour word is unreliable (teal reads `cyan` / `black`, grey-olive reads `green`; the
  camera's white balance is latched); the embedding separates them correctly. Query by
  embedding, keep the word as a hint.
- The score cannot be a threshold: with the prompt ensemble SigLIP's sigmoid is near 1 for every
  object of the right class. "Not found" needs its own rule (margin to the second hit, or class
  plus embedding).
- Stage 3 with Grounding DINO + SAM standing in for SAM 3: crops are whole objects, the lounge
  count drops from 26 boxes to 20 against ~18–21 pieces in the map, and small objects appear
  (fire extinguisher, trash can). Getting there took four association attempts, each failure
  instructive: shared voxels miss because views hit different faces; an unbounded footprint rule
  chains a lounge into one 7 m "sofa"; nearness has to be measured in the floor plane; and each
  mask's points must first be cut to their largest 3D cluster, because a few always fall through
  a gap onto something behind.
- What limits stage 3 now is the detector: Grounding DINO calls corridors and glass doors
  "escalator" and bar stools "armchair". That is what SAM 3 is for.

## 5. SAM 3 status

Access was granted on 2026-10-18 and the backend ran unchanged on the first try. On the same
images, prompts and association it removes Grounding DINO's false positives (21 "escalator"
objects down to one, the staircase), finds every small object with a correct crop, and with
`--min-obs 4` gives 21 boxes in the teal lounge against ~18–21 pieces. It costs 10.9 s per image
on MPS for 18 prompts, 5.4× Grounding DINO. Decision: SAM 3 is the instance backend, `--min-obs
3–4`, and the prompt list is the speed knob. Numbers in `semantic-mapping.md`, "Object map".

## 6. Open items

- Prompt list per site, to bring SAM 3's time down; and Jetson feasibility (unmeasured).
- A "not found" rule for object queries.
- Stable object ids across re-runs (match the previous objects.json by position and class), once the backend stores ids. Duplicate ids within one map are handled (`dedup`, see semantic-mapping.md).
- Keepout-mask generation from `map_semantic_clean.pcd` (section 2, item 1), checked on the
  glass façade and the escalator of this map.
- Send one object's goal pose to `navigate_to_goal` on the robot or in simulation and judge where
  it stops; not verifiable offline.
- `stamps.txt` beside `poses.txt` in `save_maps`.
- Camera extrinsic calibration (`direct_visual_lidar_calibration`) before judging thin objects.
- The Workspace's `build/` disappeared mid-run on 2026-10-17 and `install/` is dangling symlinks;
  the Workspace needs a rebuild (the replay used a scratch build of `syncai_common` and
  `clean_map`).

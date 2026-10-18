#!/usr/bin/env bash
# Offline semantic map from a replayed bag, once per 2D backend:
#   keyframe stamps -> per-patch labels -> voxel fusion (raw + masked by the cleaned map.pcd) -> renders
#
#   run_all.sh <bag_dir(uncompressed view)> <map_dir> <replay_dir> <out_root> [model[:stride] ...]
#
# Each model writes to <out_root>/<model tag>/ (segment.model_tag); the default list is the seven
# backends compared in docs/semantic-mapping.md. A ":N" suffix labels every N-th keyframe (the
# open-vocabulary backend is ~10x slower than the rest). Then:
#   .venv/bin/python compare.py <map_dir> <out_root>/compare <out_root>/<tag>...
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
PY=$HERE/.venv/bin/python
BAG=$1; MAP=$2; REPLAY=$3; OUT=$4; shift 4
MODELS=("$@")
if [ ${#MODELS[@]} -eq 0 ]; then
  MODELS=(
    facebook/mask2former-swin-large-ade-semantic
    facebook/mask2former-swin-tiny-ade-semantic
    nvidia/segformer-b5-finetuned-ade-640-640
    nvidia/segformer-b2-finetuned-ade-512-512
    shi-labs/oneformer_ade20k_swin_large
    facebook/mask2former-swin-large-coco-panoptic
    grounded-sam:3
  )
fi
QUIET='Warning|warn|Loading weights|LOAD REPORT|UNEXPECTED|MISSING|Notes|^-|^Key|^\s*$|Redirects|relative_position|layernorm|slow image processor|use_fast|Device set'
mkdir -p "$OUT"
cd "$HERE"
[ -s "$OUT/keyframes.txt" ] || $PY keyframes.py "$REPLAY" "$MAP" "$OUT/keyframes.txt"
for spec in "${MODELS[@]}"; do
  model=${spec%%:*}
  stride=1; [[ "$spec" == *:* ]] && stride=${spec##*:}
  tag=$($PY -c "from segment import model_tag; print(model_tag('$model'))")
  run=$OUT/$tag
  mkdir -p "$run"
  echo "=== $model -> $run (stride $stride) $(date)"
  $PY label_patches.py "$BAG" "$MAP" "$OUT/keyframes.txt" "$REPLAY/lio_odom.txt" "$run" \
      --model "$model" --stride "$stride" 2>&1 | grep -v -E "$QUIET" | tee "$run/label.log"
  $PY fuse.py "$MAP" "$run" "$run" | tee "$run/fuse.log"
  $PY render.py "$run"
  # the masked variant needs clean_map's map.pcd; when the clean is not done yet, run fuse_clean.sh later
  if [ "$(sed -n 's/.*"status":"\([a-z]*\)".*/\1/p' "$MAP/map_clean.recipe.json" 2>/dev/null)" = ok ]; then
    $PY fuse.py "$MAP" "$run" "$run" --mask-with "$MAP/map.pcd" --name map_semantic_clean | tee "$run/fuse_clean.log"
    $PY render.py "$run" map_semantic_clean
  else
    echo "map_clean.recipe.json is not ok: skipping the masked fusion for $tag"
  fi
done
echo "=== all done $(date)"

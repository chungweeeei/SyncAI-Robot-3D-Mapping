#!/usr/bin/env bash
# Offline semantic map from a replayed bag: keyframe stamps -> per-patch labels -> voxel fusion.
#   run_all.sh <bag_dir(uncompressed view)> <map_dir> <replay_dir> <out_dir> [model]
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
PY=$HERE/.venv/bin/python
BAG=$1; MAP=$2; REPLAY=$3; OUT=$4; MODEL=${5:-facebook/mask2former-swin-large-ade-semantic}
mkdir -p "$OUT"
cd "$HERE"
$PY keyframes.py "$REPLAY" "$MAP" "$OUT/keyframes.txt"
$PY label_patches.py "$BAG" "$MAP" "$OUT/keyframes.txt" "$REPLAY/lio_odom.txt" "$OUT" --model "$MODEL" 2>&1 | grep -v -E "Warning|Loading weights|LOAD REPORT|UNEXPECTED|MISSING|Notes|^-|^Key|^\s*$|Redirects|relative_position|layernorm"
$PY fuse.py "$MAP" "$OUT" "$OUT"

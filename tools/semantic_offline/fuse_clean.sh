#!/usr/bin/env bash
# The masked fusion + render for every run under <out_root>, once clean_map's map.pcd is in place:
#   fuse_clean.sh <map_dir> <out_root>
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
PY=$HERE/.venv/bin/python
MAP=$1; OUT=$2
cd "$HERE"
for run in "$OUT"/*/; do
  [ -s "$run/classes.json" ] || continue
  echo "=== $(basename "$run")"
  $PY fuse.py "$MAP" "$run" "$run" --mask-with "$MAP/map.pcd" --name map_semantic_clean | tee "$run/fuse_clean.log"
  $PY render.py "$run" map_semantic_clean
done

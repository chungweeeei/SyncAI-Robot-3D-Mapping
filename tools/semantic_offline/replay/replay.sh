#!/usr/bin/env bash
# Replay a bag through pointlio + pgo_node in the robot image, save with patches, wait for
# clean_map, and make sure octomap.bt exists (the viewer needs it).
#
#   replay.sh [<bag dir, container path>] [<map dir, container path>]
#
# Defaults are the 2026-10-16 run (record/dp1f_with_image_db3 -> map/dp1f_with_image). For a
# zstd file-compressed bag point the first argument at a view directory the host-side
# feed_bag.sh keeps fed (see README, "Run"): the player then only ever sees uncompressed .db3
# files, one ahead of the one it reads.
set +u
WS=/home/syncrobotic/robot_ws
OUT=/scratch/replay
BAG=${1:-$WS/record/dp1f_with_image_db3}
MAP=${2:-$WS/map/dp1f_with_image}
INI=$WS/config/instances/robot01.ini
mkdir -p "$OUT" /tmp/ros_log "$MAP"
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
cd $WS
# Fail before the 13-minute play, not after it: the save call needs the interface package's
# Python typesupport, and a workspace whose build/ was deleted leaves install/syncai_common as
# dangling --symlink-install links ("The passed service type is invalid" from ros2 service call).
# Rebuild it (into a scratch prefix is enough: colcon build --base-paths src/syncai_common
# --build-base /scratch/ifc/build --install-base /scratch/ifc/install) and source that first.
if ! python3 -c "from syncai_common.srv import SaveMaps" 2>/dev/null; then
  echo "syncai_common Python typesupport is not importable; rebuild it before replaying" >&2
  exit 2
fi
echo "start $(date)"
ros2 launch syncai_pointlio pointlio.launch.py system_config:=$INI > $OUT/pointlio.log 2>&1 &
sleep 4
ros2 launch syncai_mapping mapping.launch.py system_config:=$INI start_on_launch:=true > $OUT/pgo.log 2>&1 &
sleep 4
python3 /scratch/recorder.py $OUT > $OUT/recorder.log 2>&1 &
sleep 4
ros2 topic list > $OUT/topics.txt
ros2 bag play "$BAG" --topics /robot01/livox/lidar /robot01/livox/imu > $OUT/play.log 2>&1
echo "play done $(date)"
sleep 20
ros2 service call /robot01/pgo/save_maps syncai_common/srv/SaveMaps "{file_path: '$MAP', save_patches: true}" > $OUT/save.log 2>&1
cat $OUT/save.log
s=none
for i in $(seq 1 240); do
  s=$(python3 -c "import json; print(json.load(open('$MAP/map_clean.recipe.json'))['status'])" 2>/dev/null || echo none)
  if [ "$s" != "converting" ] && [ "$s" != "none" ]; then break; fi
  sleep 10
done
echo "clean status: $s $(date)"
pkill -INT -f recorder.py; sleep 3
pkill -INT -x pgo_node; pkill -INT -x pointlio_node; sleep 5
pkill -INT -f "ros2 launch"; sleep 2
# map_clean_octomap is off in mapping_params.yaml, so the save's clean writes no tree; the
# by-hand rerun starts from patches/ and reproduces map.pcd exactly (README, "Semantic octomap viewer").
if [ ! -s "$MAP/octomap.bt" ]; then
  echo "octomap.bt not written by the save's clean; running clean_map --octomap 1 $(date)"
  ros2 run syncai_mapping clean_map "$MAP" --octomap 1 > $OUT/clean_octomap.log 2>&1
  tail -3 $OUT/clean_octomap.log
fi
echo "done $(date)"

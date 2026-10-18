#!/usr/bin/env bash
# Replay dp1f_with_image through pointlio + pgo_node in the robot image, save with patches, wait for clean_map.
set +u
WS=/home/syncrobotic/robot_ws
OUT=/scratch/replay
MAP=$WS/map/dp1f_with_image
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
ros2 bag play record/dp1f_with_image_db3 --topics /robot01/livox/lidar /robot01/livox/imu > $OUT/play.log 2>&1
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
echo "done $(date)"

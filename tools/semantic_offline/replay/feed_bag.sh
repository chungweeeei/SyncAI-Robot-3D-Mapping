#!/usr/bin/env bash
# Host side of replaying a zstd file-compressed bag without inflating all of it: keep the file
# after the one the player has open decompressed, and delete the one before it.
#
#   feed_bag.sh <container> <view dir> <compressed bag dir> <bag base name> [<n files>]
#
# <view dir> starts with the bag's metadata.yaml rewritten for an uncompressed bag
# (compression_format / compression_mode "", .db3 names) plus files 0 and 1 inflated; the player
# in <container> reads it as /scratch/<view dir name>. Which file is open is read off the
# container's /proc/*/fd (the sqlite storage keeps it open), so the schedule follows the player
# rather than the clock. Peak disk use is two files (~4 GB for a 30 Hz camera bag) instead of
# the whole bag (14 GB for 20 minutes). Exits when the container stops.
CID=$1; VIEW=$2; SRC=$3; BASE=$4; N=${5:-$(ls "$SRC"/*.db3.zstd | wc -l | tr -d ' ')}
cur=-1
while [ "$(docker inspect -f '{{.State.Running}}' "$CID" 2>/dev/null)" = true ]; do
  open=$(docker exec "$CID" sh -c "ls -l /proc/*/fd 2>/dev/null | grep -o '${BASE}_[0-9]*\.db3' | sort -u | tail -1")
  i=${open##*_}; i=${i%.db3}
  if [ -n "$i" ] && [ "$i" != "$cur" ]; then
    cur=$i; echo "$(date +%T) file $i open"
    nxt=$((i+1))
    if [ "$nxt" -lt "$N" ] && [ ! -s "$VIEW/${BASE}_$nxt.db3" ]; then
      zstd -d -q -f -o "$VIEW/${BASE}_$nxt.db3" "$SRC/${BASE}_$nxt.db3.zstd" && echo "$(date +%T) decompressed $nxt"
    fi
    prv=$((i-1))
    if [ "$prv" -ge 0 ] && [ -e "$VIEW/${BASE}_$prv.db3" ]; then
      rm -f "$VIEW/${BASE}_$prv.db3" && echo "$(date +%T) removed $prv"
    fi
  fi
  sleep 3
done
echo "$(date +%T) container stopped"

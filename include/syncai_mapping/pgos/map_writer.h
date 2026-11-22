#pragma once
#include <filesystem>
#include <vector>

#include "syncai_mapping/pgos/pose_graph_optimizer.h"
#include "syncai_mapping/pgos/sessions.h"

/**
 * Writes every keyframe's body cloud, in its global pose, as one binary PCD at `path`: the same
 * bytes pcl::io::savePCDFileBinary writes for the merged cloud, without ever holding the merge.
 *
 * The merge used to be built in memory first, a full world-frame copy of every keyframe (32 B a
 * point, as large as the keyframes themselves), on top of the keyframes, at the moment a run is
 * largest and an OOM loses it. Here the point count is summed first, so the header is exact
 * before any data, and each keyframe is transformed into one reused buffer and appended: the
 * extra memory is one keyframe.
 *
 * Atomic: written to `<path>.save.tmp`, then renamed over `path`, so a failure or a kill leaves
 * the previous file (or none), never a truncated one. Not `<path>.tmp`, which is clean_map's.
 * Throws std::runtime_error (the tmp file removed) on any failure.
 */
void writeMergedMapPcd(
  const std::filesystem::path & path, const std::vector<KeyPoseWithCloud> & key_poses);

/**
 * Writes the keyframes themselves into `dir`, the input clean_map and a later resume_mapping read:
 *
 *   patches/<i>.pcd   keyframe i's body cloud (binary PCD), `patches/` created when missing
 *   poses.txt         one line per keyframe, "<i>.pcd tx ty tz qw qx qy qz", its global pose
 *   sessions.json     only for a map of more than one mapping run (a resumed one), see below
 *
 * `session_starts` holds the index of each run's first keyframe, ascending, the first 0
 * (PoseGraphOptimizer::sessionStarts()). poses.txt is one sequence across the runs, and its
 * readers (the planner's stair links above all) take consecutive rows for consecutive steps of
 * one walk, which is false across a run boundary: the robot was carried, or walked unmapped, to
 * where the next run began. sessions.json is that boundary on disk:
 *
 *   {"version": 1, "sessions": [{"first": 0, "count": 812}, {"first": 812, "count": 240}]}
 *
 * No name and no path in it, like everything else in a map directory. A run that added no
 * keyframe is left out, and a map of one run has NO sessions.json: a stale one from an earlier
 * save into `dir` is removed, so the file always describes the poses.txt beside it.
 *
 * Does not remove an existing `patches/`: the caller does, because it also has a running clean
 * to stop first. Throws (pcl::IOException, std::runtime_error) on any failure, with what was
 * written so far left in place.
 */
void writeKeyframes(
  const std::filesystem::path & dir, const std::vector<KeyPoseWithCloud> & key_poses,
  const std::vector<size_t> & session_starts);

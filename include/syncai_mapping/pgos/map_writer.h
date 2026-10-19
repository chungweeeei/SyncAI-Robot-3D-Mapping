#pragma once
#include <filesystem>
#include <vector>

#include "syncai_mapping/pgos/pose_graph_optimizer.h"

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

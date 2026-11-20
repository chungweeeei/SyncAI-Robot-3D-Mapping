#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "syncai_mapping/pgos/pose_graph_optimizer.h"

/**
 * A map directory read back as keyframes, for resume_mapping: what save_maps wrote with
 * save_patches (see writeKeyframes in map_writer.h), and nothing else of it. map.pcd is not read
 * (clean_map has rewritten it without the people the patches still hold, and it has no scan
 * origins); the patches are the map.
 *
 * A keyframe comes back with its global pose and its body cloud only. Its LIO pose, stamp and
 * path length were never saved and stay at their defaults: PoseGraphOptimizer::loadPrior() holds
 * such keyframes as constants and never reads those fields.
 */
struct SavedMap
{
  std::vector<KeyPoseWithCloud> key_poses;
  // First keyframe of each mapping run, ascending, the first 0: sessions.json, or {0} without
  // one.
  std::vector<size_t> session_starts;
  // Not empty when sessions.json was there and unusable (the map then loads as one session).
  std::string warning;
};

/**
 * Throws std::runtime_error, with a sentence fit for the operator, when `dir` is not a map that
 * can be resumed: no poses.txt (saved without patches), a row that does not parse, a patch that
 * is missing or unreadable, or no keyframe at all. Strict on purpose, where clean_map skips a bad
 * row: a map resumed short of some keyframes is saved short of them, under a new name, with
 * nothing to say so.
 *
 * A pose is read as printed, 6 significant digits, so a keyframe's translation survives any
 * number of resume + save rounds unchanged and its quaternion to the last digit.
 */
SavedMap readSavedMap(const std::filesystem::path & dir);

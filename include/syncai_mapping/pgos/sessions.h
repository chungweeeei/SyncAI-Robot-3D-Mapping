#pragma once
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

/**
 * sessions.json: beside poses.txt, where each mapping run begins in it (writeKeyframes in
 * map_writer.h has the format and why it exists).
 *
 * Its own library (`map_sessions`: the standard library and nlohmann/json, no PCL or GTSAM),
 * because it has readers on both sides of the save: pgo_node's resume_mapping (map_reader) and
 * clean_map (dynamic_removal/remover), which must not link the pose graph.
 */
inline constexpr char kSessionsFile[] = "sessions.json";

/**
 * Writes `dir`/sessions.json for `total` keyframes whose runs begin at `session_starts`
 * (ascending, the first 0), through a .tmp + rename. A run that banked no keyframe is left out.
 * One run writes NO file and removes a stale one, so the file always describes the poses.txt
 * beside it. Throws std::runtime_error when the write fails.
 */
void writeSessions(
  const std::filesystem::path & dir, size_t total, const std::vector<size_t> & session_starts);

/**
 * sessions.json at `path` -> the first keyframe of each run, for a poses.txt of `total` rows.
 * False, with `why` and `starts` untouched, for anything but an exact tiling of [0, total): half
 * a boundary list is worse than none, and every reader then takes the map for one run.
 */
bool readSessions(
  const std::filesystem::path & path, size_t total, std::vector<size_t> & starts,
  std::string & why);

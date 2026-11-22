#pragma once
#include <filesystem>
#include <optional>
#include <string>

/**
 * The pose syncai_localizer last saved (SyncAI-Robot-Workspace, localizers/last_pose_store.h):
 * `config/state/<robot_id>/last_pose.json`, written during navigation from every trusted
 * registration, at most every few seconds, the parked pose included once the robot has stood for
 * that long. It is where the robot was when the nav session ended, which is where it still is
 * when the operator switches to mapping to resume the map they were navigating on.
 *
 *   {"map": "dp", "x": .., "y": .., "ground_z": .., "yaw": .., "saved_at": "<UTC>"}
 *
 * `map` is the map directory's NAME ([map] name), x / y / yaw the lidar body in that map's frame
 * (what a resume hint is), ground_z the ground under it (body z - lidar height).
 *
 * This is another package's file and this is its second reader (the first is that package's own
 * launch): a change of its fields is a change here. Read-only, and never trusted by itself: the
 * robot may have been driven in the mapping session, or carried, since it was written, so what
 * it says is a hint to verify by registration (SessionAlignRequest::hint_radius), not a pose.
 */
struct LastPoseFile
{
  std::string map;
  double x = 0.0;
  double y = 0.0;
  double ground_z = 0.0;
  double yaw = 0.0;
  std::string saved_at;  // informational
};

// nullopt, with `why`, when the file is missing, not JSON, or lacks any of map / x / y /
// ground_z / yaw: a truncated file must never become a hint. The same rules as the localizer's
// loadLastPose.
std::optional<LastPoseFile> readLastPoseFile(const std::filesystem::path & path, std::string & why);

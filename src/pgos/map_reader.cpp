#include "syncai_mapping/pgos/map_reader.h"

#include <pcl/io/pcd_io.h>

#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include "nlohmann/json.hpp"
#include "syncai_mapping/pgos/map_writer.h"

namespace
{
namespace fs = std::filesystem;

// sessions.json -> the first keyframe of each run. Anything but an exact tiling of [0, total) is
// refused whole: half a boundary list is worse than none.
bool readSessions(
  const fs::path & path, size_t total, std::vector<size_t> & starts, std::string & why)
{
  std::ifstream in(path);
  if (!in) {
    why = "cannot open it";
    return false;
  }
  const nlohmann::json doc = nlohmann::json::parse(in, nullptr, false);
  if (doc.is_discarded() || !doc.is_object()) {
    why = "not a JSON object";
    return false;
  }
  if (!doc.contains("version") || !doc["version"].is_number_integer() || doc["version"] != 1) {
    why = "not version 1";
    return false;
  }
  if (!doc.contains("sessions") || !doc["sessions"].is_array() || doc["sessions"].empty()) {
    why = "no sessions";
    return false;
  }
  size_t next = 0;
  std::vector<size_t> ret;
  for (const nlohmann::json & s : doc["sessions"]) {
    if (
      !s.is_object() || !s.contains("first") || !s.contains("count") ||
      !s["first"].is_number_unsigned() || !s["count"].is_number_unsigned()) {
      why = "a session without first / count";
      return false;
    }
    const size_t first = s["first"].get<size_t>();
    const size_t count = s["count"].get<size_t>();
    if (first != next || count == 0) {
      why = "the sessions do not follow one another";
      return false;
    }
    ret.push_back(first);
    next = first + count;
  }
  if (next != total) {
    why = "the sessions cover " + std::to_string(next) + " keyframes, poses.txt has " +
          std::to_string(total);
    return false;
  }
  starts = std::move(ret);
  return true;
}
}  // namespace

SavedMap readSavedMap(const fs::path & dir)
{
  std::error_code ec;
  if (!fs::is_directory(dir, ec))
    throw std::runtime_error("No map directory at " + dir.string() + ".");
  const fs::path poses_path = dir / "poses.txt";
  std::ifstream poses(poses_path);
  if (!poses)
    throw std::runtime_error(
      "This map has no poses.txt: it was saved without its keyframes and cannot be resumed.");

  SavedMap map;
  std::string line;
  size_t line_no = 0;
  while (std::getline(poses, line)) {
    line_no++;
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    const std::string where = "poses.txt line " + std::to_string(line_no);

    std::istringstream row(line);
    row.imbue(std::locale::classic());
    std::string name;
    double tx, ty, tz, qw, qx, qy, qz;
    if (!(row >> name >> tx >> ty >> tz >> qw >> qx >> qy >> qz))
      throw std::runtime_error("The map is damaged: " + where + " is not a keyframe pose.");
    // A bare basename, as save_maps writes it: the name is joined onto patches/ below.
    if (name.find('/') != std::string::npos || name == "." || name == "..")
      throw std::runtime_error("The map is damaged: " + where + " names a path, not a patch.");
    Eigen::Quaterniond q(qw, qx, qy, qz);
    if (!(q.norm() > 1e-6))
      throw std::runtime_error("The map is damaged: " + where + " has no rotation.");

    KeyPoseWithCloud kp;
    kp.r_global = q.normalized().toRotationMatrix();
    kp.t_global = V3D(tx, ty, tz);
    kp.body_cloud.reset(new CloudType);
    const fs::path patch = dir / "patches" / name;
    if (
      !fs::is_regular_file(patch, ec) || pcl::io::loadPCDFile(patch.string(), *kp.body_cloud) != 0)
      throw std::runtime_error(
        "The map is damaged: patches/" + name + " (" + where + ") is missing or unreadable.");
    map.key_poses.push_back(std::move(kp));
  }
  if (map.key_poses.empty()) throw std::runtime_error("The map has no keyframes.");

  map.session_starts = {0};
  const fs::path sessions_path = dir / kSessionsFile;
  if (fs::exists(sessions_path, ec)) {
    std::string why;
    if (!readSessions(sessions_path, map.key_poses.size(), map.session_starts, why))
      map.warning =
        std::string(kSessionsFile) + " ignored (" + why + "); the map is read as one run";
  }
  return map;
}

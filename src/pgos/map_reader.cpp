#include "syncai_mapping/pgos/map_reader.h"

#include <pcl/io/pcd_io.h>

#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include "syncai_mapping/pgos/sessions.h"

namespace fs = std::filesystem;

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

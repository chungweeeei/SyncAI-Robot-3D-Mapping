#include "syncai_mapping/pgos/last_pose_file.h"

#include <fstream>
#include <utility>

#include "nlohmann/json.hpp"

std::optional<LastPoseFile> readLastPoseFile(const std::filesystem::path & path, std::string & why)
{
  std::ifstream in(path);
  if (!in) {
    why = "cannot open it";
    return std::nullopt;
  }
  const nlohmann::json doc = nlohmann::json::parse(in, nullptr, false);
  if (doc.is_discarded() || !doc.is_object()) {
    why = "not a JSON object";
    return std::nullopt;
  }
  LastPoseFile pose;
  if (!doc.contains("map") || !doc["map"].is_string() || doc["map"].get<std::string>().empty()) {
    why = "no map name";
    return std::nullopt;
  }
  pose.map = doc["map"].get<std::string>();
  for (const auto & [key, out] :
       {std::pair<const char *, double *>{"x", &pose.x},
        {"y", &pose.y},
        {"ground_z", &pose.ground_z},
        {"yaw", &pose.yaw}}) {
    if (!doc.contains(key) || !doc[key].is_number()) {
      why = std::string("no number for ") + key;
      return std::nullopt;
    }
    *out = doc[key].get<double>();
  }
  if (doc.contains("saved_at") && doc["saved_at"].is_string())
    pose.saved_at = doc["saved_at"].get<std::string>();
  return pose;
}

#include "syncai_mapping/dynamic_removal/recipe.hpp"

#include <unistd.h>

#include <ctime>
#include <fstream>
#include <stdexcept>
#include <system_error>

#include "nlohmann/json.hpp"

namespace syncai_mapping::dynamic_removal::recipe
{

namespace
{

// ordered_json: fields come out in insertion order, the order documented in
// the header (status, started_at, finished_at, params, then measurements or
// error). nlohmann formats numbers locale-independently and as the shortest
// text that round-trips, and writes NaN / inf as null rather than invalid
// JSON.
using Json = nlohmann::ordered_json;

Json head(const char * status, const std::string & started_at)
{
  Json j;
  j["status"] = status;
  j["started_at"] = started_at;
  return j;
}

Json paramsJson(const Params & p)
{
  Json j;
  j["voxel_size"] = p.voxel_size;
  j["max_range"] = p.max_range;
  j["min_range"] = p.min_range;
  j["lidar_height"] = p.lidar_height;
  j["floor_band"] = p.floor_band;
  j["floor_band_dynamic"] = p.floor_band_dynamic;
  j["dynamic_max_height"] = p.dynamic_max_height;
  j["floor_radius"] = p.floor_radius;
  j["sparse_min_hits"] = p.sparse_min_hits;
  j["dynamic_miss_ratio"] = p.dynamic_miss_ratio;
  j["dynamic_min_miss"] = p.dynamic_min_miss;
  j["clip_below_floor"] = p.clip_below_floor;
  j["octomap"] = p.octomap;
  j["floor_fill_radius"] = p.floor_fill_radius;
  return j;
}

// Compact, one line. `replace` instead of the default `strict`: an error text
// from the OS or a library is not guaranteed to be valid UTF-8, and a status
// write must not throw over it.
std::string dump(const Json & j)
{
  return j.dump(-1, ' ', false, Json::error_handler_t::replace);
}

}  // namespace

std::string isoUtcNow()
{
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

std::string converting(const Params & params, const std::string & started_at)
{
  Json j = head("converting", started_at);
  j["params"] = paramsJson(params);
  return dump(j);
}

std::string ok(
  const Params & params, const std::string & started_at, const std::string & finished_at,
  const Measurements & m)
{
  Json j = head("ok", started_at);
  j["finished_at"] = finished_at;
  j["params"] = paramsJson(params);
  Json & mj = j["measurements"];
  mj["keyframes"] = m.keyframes;
  mj["skipped_patches"] = m.skipped_patches;
  mj["points"] = m.points;
  mj["clipped_rays"] = m.clipped_rays;
  mj["dynamic_voxels"] = m.dynamic_voxels;
  mj["sparse_voxels"] = m.sparse_voxels;
  mj["map_points_removed"] = m.map_points_removed;
  mj["map_points_kept"] = m.map_points_kept;
  mj["octomap_written"] = m.octomap_written;
  mj["octomap_leaves"] = m.octomap_leaves;
  mj["octomap_occupied_leaves"] = m.octomap_occupied_leaves;
  mj["octomap_free_leaves"] = m.octomap_free_leaves;
  mj["octomap_unknown_voxels"] = m.octomap_unknown_voxels;
  mj["octomap_floor_restored"] = m.octomap_floor_restored;
  mj["octomap_floor_filled"] = m.octomap_floor_filled;
  if (!m.octomap_error.empty()) mj["octomap_error"] = m.octomap_error;
  mj["elapsed_s"] = m.elapsed_s;
  return dump(j);
}

std::string failed(
  const Params & params, const std::string & started_at, const std::string & finished_at,
  const std::string & error)
{
  std::string line = error;
  for (char & c : line) {
    if (c == '\n' || c == '\r') c = ' ';
  }
  Json j = head("failed", started_at);
  j["finished_at"] = finished_at;
  j["params"] = paramsJson(params);
  j["error"] = line;
  return dump(j);
}

void writeAtomic(const std::filesystem::path & dir, const std::string & json)
{
  const std::filesystem::path final_path = dir / kSidecarFile;
  // Per-process temp name: pgo_node and the clean it spawned both write this
  // file, and two writers sharing one .tmp would rename each other's file
  // away (the loser then fails with ENOENT).
  const std::filesystem::path tmp_path =
    dir / (std::string(kSidecarFile) + "." + std::to_string(getpid()) + ".tmp");
  {
    std::ofstream f(tmp_path, std::ios::trunc);
    f << json << "\n";
    f.close();
    if (!f) {
      std::error_code ec;
      std::filesystem::remove(tmp_path, ec);
      throw std::runtime_error(std::string("cannot write ") + kSidecarFile);
    }
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, final_path, ec);
  if (ec) {
    std::error_code rm_ec;
    std::filesystem::remove(tmp_path, rm_ec);
    throw std::runtime_error(std::string("cannot rename ") + kSidecarFile + ": " + ec.message());
  }
}

}  // namespace syncai_mapping::dynamic_removal::recipe

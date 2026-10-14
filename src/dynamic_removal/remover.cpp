#include "syncai_mapping/dynamic_removal/remover.hpp"

#include <octomap/octomap.h>
#include <pcl/common/transforms.h>
#include <pcl/exceptions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/stat.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

#include "syncai_mapping/dynamic_removal/edits.hpp"

namespace syncai_mapping::dynamic_removal
{

namespace
{

namespace fs = std::filesystem;

struct PoseLine
{
  std::string name;
  double tx, ty, tz, qw, qx, qy, qz;
};

// A keyframe's xy and the floor height under it (z - lidar_height).
struct FloorRef
{
  double x, y, f;
};

void readPoses(
  const fs::path & poses_path, double lidar_height, std::vector<PoseLine> & poses,
  std::vector<FloorRef> & floors)
{
  std::ifstream in(poses_path);
  if (!in) throw InputError("cannot open poses.txt (saved without patches?)");
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    PoseLine p;
    if (ss >> p.name >> p.tx >> p.ty >> p.tz >> p.qw >> p.qx >> p.qy >> p.qz) {
      poses.push_back(p);
      floors.push_back({p.tx, p.ty, p.tz - lidar_height});
    }
  }
  if (poses.empty()) throw InputError("poses.txt lists no keyframe");
}

// "The save this clean started from": a new save rewrites poses.txt. Size and
// mtime too, because a truncating rewrite may keep the inode.
struct Fingerprint
{
  dev_t dev = 0;
  ino_t ino = 0;
  off_t size = 0;
  timespec mtime = {};
  bool operator==(const Fingerprint & o) const
  {
    return dev == o.dev && ino == o.ino && size == o.size && mtime.tv_sec == o.mtime.tv_sec &&
           mtime.tv_nsec == o.mtime.tv_nsec;
  }
};

Fingerprint fingerprint(const fs::path & poses_path)
{
  struct stat st = {};
  if (stat(poses_path.c_str(), &st) != 0) return {};
  Fingerprint f;
  f.dev = st.st_dev;
  f.ino = st.st_ino;
  f.size = st.st_size;
  f.mtime = st.st_mtim;
  return f;
}

// The raw body-frame patch and its pose; false when unreadable.
bool loadPatch(
  const fs::path & dir, const PoseLine & p, pcl::PointCloud<pcl::PointXYZI> & cloud,
  Eigen::Quaterniond & q, Eigen::Vector3d & t)
{
  try {
    if (pcl::io::loadPCDFile((dir / "patches" / p.name).string(), cloud) < 0) return false;
  } catch (const pcl::PCLException &) {
    return false;
  }
  q = Eigen::Quaterniond(p.qw, p.qx, p.qy, p.qz);
  q.normalize();
  t = Eigen::Vector3d(p.tx, p.ty, p.tz);
  return true;
}

// One keyframe's world returns beyond min_range, split into normal ones and
// those below the floor (clip_below_floor), whose ray stops at the clip
// height. Both counting passes build it the same way, or a voxel's hits and
// misses would come from different scans.
struct Scan
{
  octomap::Pointcloud endpoints;
  std::vector<octomap::point3d> clipped_ends;  // where a clipped ray stops
  std::vector<octomap::point3d> clipped_hits;  // the clipped returns themselves
};

Scan worldScan(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const Eigen::Quaterniond & q,
  const Eigen::Vector3d & t, const Params & params)
{
  Scan s;
  s.endpoints.reserve(cloud.size());
  // One voxel inside the floor band, not at its edge: grazing rays carve all
  // the way down to the clip.
  const double clip_z =
    t.z() - params.lidar_height - std::max(0.0, params.floor_band - params.voxel_size);
  for (const auto & pt : cloud) {
    const Eigen::Vector3d b(pt.x, pt.y, pt.z);
    const double r = b.norm();
    if (!std::isfinite(r) || r < params.min_range) continue;
    const Eigen::Vector3d w = q * b + t;
    if (params.clip_below_floor && w.z() < clip_z) {
      const double f = (t.z() - clip_z) / (t.z() - w.z());
      const Eigen::Vector3d c = t + f * (w - t);
      s.clipped_ends.emplace_back(c.x(), c.y(), c.z());
      s.clipped_hits.emplace_back(w.x(), w.y(), w.z());
      continue;
    }
    s.endpoints.push_back(w.x(), w.y(), w.z());
  }
  return s;
}

// The voxels one scan crosses (free_cells) and ends in (occupied_cells):
// OctoMap's discretized update truncated at max_range (so occupied matches
// pass 1's hit keys exactly), plus the clipped rays up to their clip point.
// Occupied wins within a scan, as in OctoMap.
void castScan(
  octomap::OcTree & tree, const Scan & s, const octomap::point3d & origin, double max_range,
  octomap::KeySet & free_cells, octomap::KeySet & occupied_cells)
{
  tree.computeDiscreteUpdate(s.endpoints, origin, free_cells, occupied_cells, max_range);
  if (!s.clipped_ends.empty()) {
    octomap::KeyRay ray;
    for (octomap::point3d c : s.clipped_ends) {
      const octomap::point3d d = c - origin;
      if (max_range > 0.0 && d.norm() > max_range) c = origin + d.normalized() * max_range;
      if (tree.computeRayKeys(origin, c, ray)) free_cells.insert(ray.begin(), ray.end());
    }
  }
  for (const auto & k : occupied_cells) free_cells.erase(k);
}

// Per-scan counts and the verdict, keyed on HIT voxels only: keying every
// crossed voxel would hold the site's free space (GBs).
struct Counts
{
  uint32_t hits = 0;
  uint32_t clipped = 0;
  uint32_t misses = 0;
  bool removed = false;
  bool on_floor = false;  // within floor_band of its local floor
};
using CountMap = std::unordered_map<octomap::OcTreeKey, Counts, octomap::OcTreeKey::KeyHash>;

fs::path tmpPath(const fs::path & dir, const char * name)
{
  return dir / (std::string(name) + ".tmp");
}

void savePcd(const fs::path & tmp, const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  // pcl's what() holds the absolute path, and this text lands in the sidecar.
  try {
    if (pcl::io::savePCDFileBinary(tmp.string(), cloud) != 0) {
      throw std::runtime_error(std::string("cannot write ") + kMapFile);
    }
  } catch (const pcl::IOException &) {
    throw std::runtime_error(std::string("cannot write ") + kMapFile);
  }
}

// Floor inferred where the lidar sampled it too thinly (floor_fill_radius):
// gaps in the hit floor closed, per column, at the height of the floor next
// to them. A column takes a floor voxel only if
//  - it has none within a level of that height already,
//  - that height is within floor_band of its local floor, with a keyframe on
//    that level within floor_fill_radius,
//  - it is enclosed: along x, y or a diagonal, hit floor or an obstacle lies
//    within kFillGap on BOTH sides (a shadow behind a shelf or along a wall).
//    Grown outward instead, the fill ran out through every door and glass
//    front. kFillGap closes gaps up to 2 * kFillGap wide; larger, it filled a
//    pool (water returns nothing) from one rim,
//  - some voxel up to kFillClearance above is observed free (in view, not
//    behind a wall or under a shelf) and none from two voxels up is occupied
//    (a floor surface often spans two); its seed must be open the same way,
//    so the fill never grows from a wall's foot,
//  - nothing kept was hit from floor_band to kFillDropDepth below (a stair
//    down, a dock edge). A hole no ray looked into is not caught: hence the
//    radius.
// It grows from hit floor only, so it never makes an island.
constexpr double kFillClearance = 0.5;  // m
constexpr double kFillDropDepth = 1.0;  // m
constexpr double kFillGap = 0.6;        // m

size_t fillFloor(
  octomap::OcTree & tree, const CountMap & counts, const std::vector<FloorRef> & floors,
  const Params & params, const std::function<double(double, double, double)> & localFloor)
{
  const double res = params.voxel_size, band = params.floor_band, r = params.floor_fill_radius;
  const int level = static_cast<int>(std::ceil(2.0 * band / res));
  const int clearance = static_cast<int>(std::ceil(kFillClearance / res));
  const int drop_top = static_cast<int>(std::ceil(band / res)) + 1;
  const int drop_bottom = static_cast<int>(std::ceil(kFillDropDepth / res));
  const float occupied_log = tree.getClampingThresMaxLog();
  auto shifted = [](const octomap::OcTreeKey & k, int dx, int dy, int dz) {
    return octomap::OcTreeKey(k[0] + dx, k[1] + dy, k[2] + dz);
  };

  // Seeds: per column and level, the kept floor voxel nearest the local
  // floor; growing from a wall's foot would raise the floor by up to
  // floor_band.
  octomap::KeySet ground, filled_keys, tried;
  std::unordered_map<uint64_t, std::pair<octomap::OcTreeKey, double>> seeds;
  for (const auto & [key, c] : counts) {
    if (c.removed || !c.on_floor || c.hits == 0) continue;
    ground.insert(key);
    const octomap::point3d v = tree.keyToCoord(key);
    const double fz = localFloor(v.x(), v.y(), v.z());
    const uint64_t id =
      (uint64_t(key[0]) << 32) | (uint64_t(key[1]) << 16) | uint64_t(uint16_t(std::lround(fz)));
    const double off = std::fabs(v.z() - fz);
    auto [it, fresh] = seeds.emplace(id, std::make_pair(key, off));
    if (!fresh && off < it->second.second) it->second = {key, off};
  }
  std::vector<octomap::OcTreeKey> queue;
  queue.reserve(seeds.size());

  // Keyframes within r of a 1 m cell, cached per cell.
  std::unordered_map<long long, std::vector<FloorRef>> near;
  auto nearPath = [&](double x, double y, double fz) {
    const long long k = ((long long)std::floor(x) << 32) ^ (long long)(std::floor(y) + 1e6);
    auto it = near.find(k);
    if (it == near.end()) {
      const double cx = std::floor(x) + .5, cy = std::floor(y) + .5, reach = r + 0.75;
      std::vector<FloorRef> v;
      for (const auto & p : floors) {
        if ((p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy) < reach * reach) v.push_back(p);
      }
      it = near.emplace(k, std::move(v)).first;
    }
    for (const auto & p : it->second) {
      if (std::fabs(p.f - fz) <= band && (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y) <= r * r) {
        return true;
      }
    }
    return false;
  };
  auto inBand = [&](const octomap::KeySet & set, const octomap::OcTreeKey & k) {
    for (int dz = -level; dz <= level; ++dz) {
      if (set.count(shifted(k, 0, 0, dz))) return true;
    }
    return false;
  };
  // Something kept was hit from two voxels to kFillClearance above.
  auto obstacle = [&](const octomap::OcTreeKey & k) {
    for (int dz = 2; dz <= clearance; ++dz) {
      const auto it = counts.find(shifted(k, 0, 0, dz));
      if (it != counts.end() && !it->second.removed && it->second.hits > 0) return true;
    }
    return false;
  };
  auto solid = [&](const octomap::OcTreeKey & k) { return inBand(ground, k) || obstacle(k); };
  auto enclosed = [&](const octomap::OcTreeKey & k) {
    constexpr int kAxis[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
    for (const auto & a : kAxis) {
      const int steps =
        static_cast<int>(kFillGap / (res * std::sqrt(double(a[0] * a[0] + a[1] * a[1]))));
      bool both = true;
      for (int side : {1, -1}) {
        bool found = false;
        for (int i = 1; i <= steps && !found; ++i) {
          found = solid(shifted(k, side * i * a[0], side * i * a[1], 0));
        }
        both = both && found;
      }
      if (both) return true;
    }
    return false;
  };
  auto openAbove = [&](const octomap::OcTreeKey & k) {
    bool seen_free = false;
    for (int dz = 1; dz <= clearance; ++dz) {
      const auto * node = tree.search(shifted(k, 0, 0, dz));
      if (!node) continue;
      if (!tree.isNodeOccupied(node)) {
        seen_free = true;
      } else if (dz > 1) {
        return false;
      }
    }
    return seen_free;
  };
  auto dropBelow = [&](const octomap::OcTreeKey & k) {
    for (int dz = drop_top; dz <= drop_bottom; ++dz) {
      const auto it = counts.find(shifted(k, 0, 0, -dz));
      if (it != counts.end() && !it->second.removed && it->second.hits + it->second.clipped > 0) {
        return true;
      }
    }
    return false;
  };

  for (const auto & [id, s] : seeds) {
    if (openAbove(s.first)) queue.push_back(s.first);
  }
  seeds.clear();

  size_t filled = 0;
  constexpr int kStep[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (size_t head = 0; head < queue.size(); ++head) {
    const octomap::OcTreeKey from = queue[head];
    for (const auto & s : kStep) {
      const octomap::OcTreeKey k = shifted(from, s[0], s[1], 0);
      // The tests depend on the scans alone, so a column is judged once.
      if (!tried.insert(k).second || inBand(ground, k) || inBand(filled_keys, k)) continue;
      const octomap::point3d v = tree.keyToCoord(k);
      const double fz = localFloor(v.x(), v.y(), v.z());
      if (std::isnan(fz) || std::fabs(v.z() - fz) > band || !nearPath(v.x(), v.y(), fz)) continue;
      if (!openAbove(k) || dropBelow(k) || !enclosed(k)) continue;
      tree.setNodeValue(k, occupied_log, true);
      filled_keys.insert(k);
      queue.push_back(k);
      ++filled;
    }
  }
  return filled;
}

bool cleanImpl(
  const fs::path & dir, const Params & params, Measurements & out, const Progress & progress)
{
  const auto t0 = std::chrono::steady_clock::now();
  const fs::path poses_path = dir / "poses.txt";
  const Fingerprint started_from = fingerprint(poses_path);
  std::vector<PoseLine> poses;
  std::vector<FloorRef> floors;
  readPoses(poses_path, params.lidar_height, poses, floors);

  // Key arithmetic and ray traversal, and with params.octomap the tree itself.
  // Not const: computeDiscreteUpdate uses scratch members.
  octomap::OcTree tree(params.voxel_size);
  const double band = params.floor_band, dyn_band = params.floor_band_dynamic,
               maxh = params.dynamic_max_height, rad = params.floor_radius;
  CountMap counts;

  // ---- Pass 1: hits ----
  //
  // The ray origin is the keyframe (IMU) pose, ~5 cm from the lidar's: under
  // one voxel, so the extrinsic is not applied.
  for (const auto & p : poses) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    if (!loadPatch(dir, p, cloud, q, t)) {
      ++out.skipped_patches;
      continue;
    }
    const Scan scan = worldScan(cloud, q, t, params);
    const octomap::point3d origin(p.tx, p.ty, p.tz);
    octomap::KeySet hit_keys, clipped_keys;  // once per scan
    for (const auto & w : scan.endpoints) {
      if (params.max_range > 0.0 && (w - origin).norm() > params.max_range) continue;
      octomap::OcTreeKey k;
      if (tree.coordToKeyChecked(w, k)) hit_keys.insert(k);
    }
    for (const auto & w : scan.clipped_hits) {
      octomap::OcTreeKey k;
      if (tree.coordToKeyChecked(w, k)) clipped_keys.insert(k);
    }
    for (const auto & k : hit_keys) ++counts[k].hits;
    for (const auto & k : clipped_keys) ++counts[k].clipped;
    out.clipped_rays += scan.clipped_ends.size();
    out.points += scan.endpoints.size() + scan.clipped_ends.size();
    ++out.keyframes;
    if (progress && !progress("hits", out.keyframes + out.skipped_patches, poses.size())) {
      return false;
    }
  }
  if (out.keyframes == 0) throw InputError("no patch under patches/ could be read");

  // ---- Pass 2: misses, and the tree ----
  //
  // After pass 1, not inside it, so a voxel first hit late still gets the
  // misses of earlier scans.
  //
  // With params.octomap each scan also goes into the tree as a discretized
  // insertPointCloud would, reusing this pass's rays. Lazy (inner nodes
  // updated once, before the write) and not pruned along the way, which
  // roughly doubled the time for a modest peak saving. Out of memory drops
  // the tree, never the clean: the 2D map is made from map.pcd.
  bool building = params.octomap;
  auto dropTree = [&](const std::string & why) {
    building = false;
    tree.clear();
    std::error_code ec;
    fs::remove(tmpPath(dir, kOctomapFile), ec);
    out.octomap_error = why;
  };
  size_t done = 0;
  for (const auto & p : poses) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    if (loadPatch(dir, p, cloud, q, t)) {
      octomap::KeySet free_cells, occupied_cells;
      castScan(
        tree, worldScan(cloud, q, t, params), octomap::point3d(p.tx, p.ty, p.tz), params.max_range,
        free_cells, occupied_cells);
      for (const auto & k : free_cells) {
        auto it = counts.find(k);
        if (it != counts.end()) ++it->second.misses;
      }
      if (building) {
        try {
          for (const auto & k : free_cells) tree.updateNode(k, false, true);
          for (const auto & k : occupied_cells) tree.updateNode(k, true, true);
        } catch (const std::bad_alloc &) {
          dropTree("out of memory building the tree; try a coarser voxel_size");
        }
      }
    }
    if (progress && !progress("misses", ++done, poses.size())) return false;
  }

  // ---- The local floor ----
  //
  // A nearby keyframe's floor, per 1 m cell within floor_radius; one global
  // level fails on sites whose floor changes height. Only keyframes whose
  // floor is at or below the voxel (within floor_band) qualify; of those the
  // highest level (within kLevelTol), and of that level the nearest in xy.
  // The xy-nearest alone is often upstairs when the voxel is under 2F.
  //
  // Up to dynamic_max_height above the nearest qualifying keyframe's floor, a
  // level competes only with a keyframe within kLevelReach of that distance:
  // "highest" is meant for a storey overhead, whose path runs nearby, not a
  // stair landing metres away, which would put a corridor's people inside
  // its floor band.
  constexpr double kLevelTol = 1.0;    // m; well under any storey height
  constexpr double kLevelReach = 3.0;  // m
  struct CellFloor
  {
    double f, d2;
  };
  std::unordered_map<long long, std::vector<CellFloor>> cell_floors;  // ascending f
  auto localFloor = [&](double x, double y, double z) {
    const long long k = ((long long)std::floor(x) << 32) ^ (long long)(std::floor(y) + 1e6);
    auto it = cell_floors.find(k);
    if (it == cell_floors.end()) {
      const double cx = std::floor(x) + .5, cy = std::floor(y) + .5;
      std::vector<CellFloor> v;
      for (const auto & p : floors) {
        const double d2 = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
        if (d2 < rad * rad) v.push_back({p.f, d2});
      }
      std::sort(
        v.begin(), v.end(), [](const CellFloor & a, const CellFloor & b) { return a.f < b.f; });
      it = cell_floors.emplace(k, std::move(v)).first;
    }
    const auto & v = it->second;
    auto hi = std::upper_bound(
      v.begin(), v.end(), z + band, [](double val, const CellFloor & c) { return val < c.f; });
    if (hi == v.begin()) return (double)NAN;
    double dmin2 = std::numeric_limits<double>::max(), fnear = NAN;
    for (auto c = v.begin(); c != hi; ++c) {
      if (c->d2 < dmin2) {
        dmin2 = c->d2;
        fnear = c->f;
      }
    }
    // Only where the voxel could be a person on the nearest keyframe's level;
    // higher up, a storey's floor far from its own path must stay floor.
    double reach2 = std::numeric_limits<double>::max();
    if (z <= fnear + maxh) {
      const double reach = std::sqrt(dmin2) + kLevelReach;
      reach2 = reach * reach;
    }
    // The highest level within reach, then its nearest keyframe (the nearest
    // qualifying keyframe is within reach, so there always is one).
    double top = NAN;
    for (auto c = hi; c != v.begin();) {
      --c;
      if (c->d2 <= reach2) {
        top = c->f;
        break;
      }
    }
    double best = std::numeric_limits<double>::max(), fz = NAN;
    for (auto c = hi; c != v.begin();) {
      --c;
      if (c->f < top - kLevelTol) break;
      if (c->d2 <= reach2 && c->d2 < best) {
        best = c->d2;
        fz = c->f;
      }
    }
    return fz;
  };

  // ---- Classify ----
  for (auto & [key, c] : counts) {
    const octomap::point3d v = tree.keyToCoord(key);
    const double fz = localFloor(v.x(), v.y(), v.z());
    bool dynamic = false;
    if (
      params.dynamic_min_miss > 0 && c.hits > 0 && c.misses >= (uint32_t)params.dynamic_min_miss &&
      c.misses >= params.dynamic_miss_ratio * c.hits) {
      dynamic = !std::isnan(fz) && v.z() >= fz + dyn_band && v.z() <= fz + maxh;
    }
    const bool on_floor = !std::isnan(fz) && std::fabs(v.z() - fz) <= band;
    c.on_floor = on_floor;
    const bool sparse = !dynamic && !on_floor && params.sparse_min_hits > 1 &&
                        c.hits + c.clipped < (uint32_t)params.sparse_min_hits;
    if (!dynamic && !sparse) continue;
    c.removed = true;
    (dynamic ? out.dynamic_voxels : out.sparse_voxels)++;
  }

  // ---- The tree: cleaned, written, freed ----
  //
  // A removed voxel becomes what it would be without its hits, so the tree
  // never claims more than was seen: free if some ray passed through it
  // (where a person stood), else UNKNOWN (node deleted) -- most sparse voxels
  // are reflections and stray returns, and free would make islands where
  // nothing was observed. Clipped-only voxels were never inserted.
  //
  // A hit floor voxel is set occupied whatever its misses: grazing rays to
  // far floor points count misses along the floor for metres, and the
  // planner would find holes everywhere.
  //
  // Freed before pass 3 so the tree and the map cloud never coexist.
  if (building) {
    try {
      const float free_log = tree.getClampingThresMinLog();
      const float occupied_log = tree.getClampingThresMaxLog();
      for (const auto & [key, c] : counts) {
        if (c.hits == 0) continue;
        if (!c.removed) {
          if (!c.on_floor) continue;
          const auto * node = tree.search(key);
          if (node && !tree.isNodeOccupied(node)) ++out.octomap_floor_restored;
          tree.setNodeValue(key, occupied_log, true);
        } else if (c.misses > 0) {
          tree.setNodeValue(key, free_log, true);
        } else {
          tree.deleteNode(key);
          ++out.octomap_unknown_voxels;
        }
      }
      if (params.floor_fill_radius > 0.0) {
        out.octomap_floor_filled = fillFloor(tree, counts, floors, params, localFloor);
      }
      tree.updateInnerOccupancy();
      // writeBinary converts to max-likelihood and prunes in place, so the
      // counts below are of what was written.
      if (tree.writeBinary(tmpPath(dir, kOctomapFile).string())) {
        for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
          ++out.octomap_leaves;
          (tree.isNodeOccupied(*it) ? out.octomap_occupied_leaves : out.octomap_free_leaves)++;
        }
        tree.clear();
      } else {
        dropTree(std::string("cannot write ") + kOctomapFile);
      }
    } catch (const std::bad_alloc &) {
      dropTree("out of memory finishing the tree; try a coarser voxel_size");
    }
  }

  // ---- Pass 3: map.pcd without the removed voxels ----
  //
  // The same concatenation pgo's saveMapsCB writes (every patch, poses.txt
  // order, no min_range cut or voxel filter) minus the removed voxels' points.
  // poses.txt's 6 significant digits are sub-millimetre at site scale.
  pcl::PointCloud<pcl::PointXYZI> map;
  done = 0;
  for (const auto & p : poses) {
    pcl::PointCloud<pcl::PointXYZI> cloud, world;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    if (loadPatch(dir, p, cloud, q, t)) {
      pcl::transformPointCloud(cloud, world, t, q);
      map.reserve(map.size() + world.size());
      for (size_t i = 0; i < cloud.size(); ++i) {
        // Pass 1's double arithmetic, so a point lands in its counted voxel.
        const auto & b = cloud[i];
        const Eigen::Vector3d w = q * Eigen::Vector3d(b.x, b.y, b.z) + t;
        octomap::OcTreeKey key;
        if (tree.coordToKeyChecked(w.x(), w.y(), w.z(), key)) {
          auto it = counts.find(key);
          if (it != counts.end() && it->second.removed) {
            ++out.map_points_removed;
            continue;
          }
        }
        map.push_back(world[i]);
      }
    }
    if (progress && !progress("map", ++done, poses.size())) return false;
  }
  out.map_points_kept = map.size();
  if (map.empty()) throw std::runtime_error("cleaned map.pcd would be empty");
  savePcd(tmpPath(dir, kMapFile), map);

  // A newer save replaced poses.txt: don't rename the OLD save's outputs over it.
  if (!(fingerprint(poses_path) == started_from)) {
    throw std::runtime_error("superseded: poses.txt changed during the clean (a newer save)");
  }
  std::error_code ec;
  fs::rename(tmpPath(dir, kMapFile), dir / kMapFile, ec);
  if (ec) throw std::runtime_error(std::string("cannot rename ") + kMapFile + ": " + ec.message());
  if (building) {
    fs::rename(tmpPath(dir, kOctomapFile), dir / kOctomapFile, ec);
    if (ec) {
      dropTree(std::string("cannot rename ") + kOctomapFile + ": " + ec.message());
    } else {
      out.octomap_written = true;
      // The new tree is the one apply_octomap_edits starts from; the copy it
      // set aside of the old one would have it patch a stale tree.
      fs::remove(dir / edits::kRawOctomapFile, ec);
    }
  }
  out.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return true;
}

}  // namespace

bool clean(
  const std::filesystem::path & dir, const Params & params, Measurements & out,
  const Progress & progress)
{
  out = Measurements{};
  auto removeTmp = [&]() {
    for (const char * name : {kMapFile, kOctomapFile}) {
      std::error_code ec;
      std::filesystem::remove(tmpPath(dir, name), ec);
    }
  };
  try {
    const bool done = cleanImpl(dir, params, out, progress);
    if (!done) removeTmp();
    return done;
  } catch (...) {
    removeTmp();
    throw;
  }
}

}  // namespace syncai_mapping::dynamic_removal

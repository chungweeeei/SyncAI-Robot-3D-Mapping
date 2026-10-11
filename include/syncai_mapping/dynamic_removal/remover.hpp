#ifndef SYNCAI_MAPPING__DYNAMIC_REMOVAL__REMOVER_HPP_
#define SYNCAI_MAPPING__DYNAMIC_REMOVAL__REMOVER_HPP_

// The post-save map cleaning: a pgo save directory (patches/<i>.pcd +
// poses.txt) ray-cast keyframe by keyframe to find the voxels that are people
// or one-off returns, and map.pcd rewritten without their points. Run by
// clean_map (dynamic_removal/main.cpp), which pgo_node spawns after a successful
// save_maps with save_patches; the README's "Cleaning map.pcd" has the
// rules and the process model.
//
// Offline on purpose (decided 2026-10-09: the cleanest final map over a clean
// live preview). A voxel is a person only once a LATER scan sees through it,
// often from another angle or on a revisit, and the judgement needs the
// final, loop-closed poses: before closure the two passes of dp1f_1008_2's
// east corridor were 1.2-2 m apart, and an online check on those poses would
// have deleted real wall at every revisit. Both exist only after the save.
//
// Prototyped as part of the OctoMap display build on
// feat/mapping-octomap-loop-noise (octomap_builder.cpp). With Params::octomap
// on it also writes octomap.bt, the occupancy tree octomap_planner plans 3D
// global paths on: the misses pass already ray-casts every scan, so the tree
// costs its memory and the node updates, not a second ray-cast. With it off,
// OctoMap is used only for its key arithmetic and ray traversal and the
// memory is the hit-voxel hash map alone.
//
// This header deliberately includes no OctoMap or PCL header: pgo_node
// includes it for Params (its ROS parameters) and the file names, and links
// neither library. Only dynamic_removal/remover.cpp does.

#include <cstddef>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>

namespace syncai_mapping::dynamic_removal
{

// pgo writes map.pcd at save time, raw; the clean REPLACES it with the same
// points minus the dynamic and sparse voxels below, via map.pcd.tmp + rename
// (an open reader keeps the old file). patches/ stay raw: they are the input,
// and a rerun by hand re-derives the cleaned map from them.
inline constexpr const char * kMapFile = "map.pcd";
// With Params::octomap: the occupancy tree of the same save, written beside
// map.pcd (octomap.bt.tmp + rename). It holds free space as well as occupied
// -- a planner must tell observed-free from never-seen, and never-seen is
// unknown (no node) -- and agrees with the cleaned map.pcd: a voxel removed
// from map.pcd is free in the tree if a ray passed through it, else unknown.
inline constexpr const char * kOctomapFile = "octomap.bt";

// One field per CLI flag, per pgo_node ROS parameter (map_clean_<name>) and
// per key of the sidecar's "params" object. Defaults are mapping_params.yaml's.
struct Params
{
  // Voxel edge (m): the unit a point is judged in.
  double voxel_size = 0.1;
  // How far from the keyframe a point is judged (m). It is the judging range,
  // not the lidar's (a Mid-360 reaches 40 m and more): a point beyond it
  // counts no hit, and its ray counts misses only up to max_range. Such a
  // point is NOT removed -- it stays in map.pcd unless a closer keyframe's
  // scan classified its voxel. Kept below the sensor's reach on purpose: far
  // returns are too sparse for the sparse rule (beam spacing outgrows a voxel)
  // and long rays turn small angular error into misses through real
  // structure; people are judged anyway from the keyframes that pass nearby.
  double max_range = 20.0;
  // Returns closer than this to the keyframe origin are ignored (m) -- the
  // robot's own body. They stay in map.pcd: nothing judged them.
  double min_range = 0.5;
  // Lidar origin above the floor (m). A keyframe's z minus this is the floor
  // under it; the rules below are cut relative to that.
  double lidar_height = 0.481;
  // The floor band (+- m around the local floor) where neither rule applies:
  // the floor is a real surface the lidar samples thinly (12 % of a
  // keyframe's points on dp1f_1008_2), so a single hit there is the norm --
  // the sparse rule took 17 % of a corridor floor before this exemption --
  // and grazing rays carve real floor voxels "free", which would read as
  // dynamic.
  double floor_band = 0.25;
  // People are looked for up to floor + dynamic_max_height (m).
  double dynamic_max_height = 2.0;
  // The local floor comes from keyframes within this radius (m) of a 1 m cell.
  double floor_radius = 10.0;

  // Counts are per scan -- a keyframe hits or crosses a voxel at most once --
  // never per ray, which would scale with 1/range^2.
  //
  // A voxel hit by fewer than sparse_min_hits keyframes is "sparse": smear, an edge
  // return, a reflection. 92 % of the occupied voxels outside one solid wall
  // of dp1f_1008_2's east corridor were single-keyframe; real wall voxels are
  // hit by a median of 4-5. <= 1 turns this off.
  int sparse_min_hits = 2;
  // A voxel hit H >= 1 times and passed through M times is "dynamic" when
  // M >= dynamic_min_miss and M >= dynamic_miss_ratio * H, and it lies
  // between floor + floor_band and floor + dynamic_max_height: a person, hit by a
  // keyframe or two and seen through by the rest. 2.0 is about OctoMap's own
  // hit / miss log-odds ratio (0.85 / 0.41). Thin static structure (a
  // handrail, a chair leg) has the same signature; measure dynamic_voxels
  // before tightening this. dynamic_min_miss <= 0 turns this off.
  double dynamic_miss_ratio = 2.0;
  int dynamic_min_miss = 3;
  // Rays that would pass more than (floor_band - voxel_size) below their own
  // keyframe's floor stop there, and their returns count as "clipped" hits
  // rather than hits. About 0.4 % of the returns on dp1f_1008_2 land under
  // the floor -- long range, half the usual intensity: specular reflections
  // off a glossy floor -- and their rays would count misses for voxels that
  // are physically under a floor. A clipped return still counts toward
  // sparse_min_hits, so a one-off reflection leaves map.pcd while a lower level seen
  // from many keyframes (stairs down) stays.
  bool clip_below_floor = true;
  // Also write kOctomapFile, at voxel_size, from the same scans (free space up
  // to max_range, occupied where a scan ended, removed voxels free when a ray
  // passed through them and unknown when none did; never-seen space has no
  // node, i.e. is unknown; a hit floor voxel stays occupied, since grazing
  // rays would otherwise carve the floor away). The
  // tree is what the clean's memory is then mostly made of: every voxel a ray
  // crossed. A tree that runs out of memory is dropped and map.pcd is still
  // cleaned; the sidecar's octomap_error says so.
  bool octomap = true;
  // Floor inferred in the tree (not in map.pcd) up to this far (m) from a
  // keyframe on the same level; <= 0 turns it off. The lidar samples the
  // floor too thinly in the shadows of shelves and walls a few metres out,
  // and a legged robot's planner needs ground under every step. Gaps in the
  // hit floor are closed where they are enclosed (hit floor or an obstacle
  // on both sides), the space above was observed free and nothing was seen
  // below (a stair down, a drop); remover.cpp's fillFloor has the rules.
  double floor_fill_radius = 3.0;
};

struct Measurements
{
  size_t keyframes = 0;        // patches read
  size_t skipped_patches = 0;  // poses.txt lines whose patch could not be read
  size_t points = 0;           // points judged (after the min_range cut)
  size_t clipped_rays = 0;     // rays stopped at the floor
  size_t dynamic_voxels = 0;
  size_t sparse_voxels = 0;
  size_t map_points_removed = 0;
  size_t map_points_kept = 0;
  // The tree as written (after pruning); zero without Params::octomap.
  bool octomap_written = false;
  size_t octomap_leaves = 0;
  size_t octomap_occupied_leaves = 0;
  size_t octomap_free_leaves = 0;
  size_t octomap_unknown_voxels = 0;  // removed voxels no ray passed through: deleted
  size_t octomap_floor_restored = 0;  // hit floor voxels the misses had made free
  size_t octomap_floor_filled = 0;    // floor voxels inferred (floor_fill_radius)
  std::string octomap_error;          // why a requested tree was not written; "" otherwise
  double elapsed_s = 0.0;
};

// (stage, keyframes done in it, keyframes in poses.txt) -> keep going? Stages
// "hits", "misses", "map". Called after every keyframe; returning false stops
// the clean with map.pcd untouched (clean() then returns false).
using Progress = std::function<bool(const char *, size_t, size_t)>;

// The input is missing or unusable (no poses.txt, no readable patch). Its
// own type so the CLI can exit 2 rather than 3.
class InputError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Read <dir>/poses.txt + <dir>/patches/, count and classify, and replace
// <dir>/map.pcd -- and with Params::octomap write <dir>/octomap.bt -- each
// via .tmp + rename, the .tmp files removed on failure.
// poses.txt is fingerprinted at the start and re-checked before the rename: a
// save into the same directory mid-clean fails this one ("superseded")
// instead of letting it rename the OLD save's outputs over the new one. Throws
// InputError or std::runtime_error, whose what() names files RELATIVE to dir
// only -- it ends up in the sidecar, which must hold no absolute path.
bool clean(
  const std::filesystem::path & dir, const Params & params, Measurements & out,
  const Progress & progress = {});

}  // namespace syncai_mapping::dynamic_removal

#endif  // SYNCAI_MAPPING__DYNAMIC_REMOVAL__REMOVER_HPP_

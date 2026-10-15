#ifndef SYNCAI_MAPPING__DYNAMIC_REMOVAL__REMOVER_HPP_
#define SYNCAI_MAPPING__DYNAMIC_REMOVAL__REMOVER_HPP_

// The post-save map cleaning: ray-cast a pgo save directory (patches/ +
// poses.txt) keyframe by keyframe and rewrite map.pcd without people and
// one-off returns. Run by clean_map (src/apps/clean_map.cpp); the README's "Cleaning
// map.pcd" has the rules and the process model.
//
// Offline on purpose: a voxel is a person only once a LATER scan sees through
// it, and judging needs the final loop-closed poses (before closure a
// revisited wall is offset and would be deleted). Both exist only after the
// save.
//
// With Params::octomap it also writes octomap.bt for octomap_planner; the
// misses pass already ray-casts every scan, so the tree costs memory, not a
// second ray-cast.
//
// No OctoMap or PCL header here: pgo_node includes this for Params and the
// file names and links neither library.

#include <cstddef>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>

namespace syncai_mapping::dynamic_removal
{

// Replaced by the clean via .tmp + rename (an open reader keeps the old file).
// patches/ stay raw: they are the input, so a rerun is idempotent.
inline constexpr const char * kMapFile = "map.pcd";
// With Params::octomap, beside map.pcd (.tmp + rename). Holds free as well as
// occupied, since a planner must tell observed-free from never-seen (unknown,
// no node). A voxel removed from map.pcd is free if a ray passed through it,
// else unknown.
inline constexpr const char * kOctomapFile = "octomap.bt";

// One field per CLI flag, per pgo_node ROS parameter (map_clean_<name>) and
// per key of the sidecar's "params". Defaults are mapping_params.yaml's.
struct Params
{
  // Voxel edge (m): the unit a point is judged in.
  double voxel_size = 0.1;
  // Judging range (m), not the lidar's. A point beyond it counts no hit and
  // its ray counts misses only up to here; it is KEPT unless a closer scan
  // classified its voxel. Below the sensor's reach because far returns are
  // too sparse for the sparse rule and long rays turn angular error into
  // misses through real structure.
  double max_range = 20.0;
  // Returns closer than this (m) are the robot's body: ignored, and kept.
  double min_range = 0.5;
  // Lidar origin above the floor (m); a keyframe's z minus this is its floor.
  double lidar_height = 0.481;
  // +- m around the local floor where the sparse rule does not apply: the
  // floor is sampled thinly, so a single hit is the norm there, and grazing
  // rays carve real floor voxels "free".
  double floor_band = 0.25;
  // The dynamic rule's narrower exemption (m above the local floor): feet and
  // ankles sit inside floor_band, and left in map.pcd they become blocked
  // discs on the robot's own track. The floor itself lies within a voxel of
  // the local floor, so it stays exempt against grazing rays.
  double floor_band_dynamic = 0.10;
  // People are looked for up to floor + dynamic_max_height (m).
  double dynamic_max_height = 2.0;
  // The local floor comes from keyframes within this radius (m) of a 1 m cell.
  double floor_radius = 10.0;

  // Counts are per scan (a keyframe hits or crosses a voxel at most once),
  // never per ray, which would scale with 1/range^2.
  //
  // A voxel hit by fewer than sparse_min_hits keyframes is "sparse": smear, an
  // edge return, a reflection (real wall voxels are hit by several). <= 1
  // turns this off.
  int sparse_min_hits = 2;
  // A voxel hit H >= 1 times and passed through M times is "dynamic" when
  // M >= dynamic_min_miss and M >= dynamic_miss_ratio * H, and it lies between
  // floor + floor_band_dynamic and floor + dynamic_max_height: a person, hit
  // by a scan or two and seen through by the rest. 2.0 is about OctoMap's own
  // hit / miss log-odds ratio. Thin static structure (a handrail, a chair leg)
  // has the same signature; measure dynamic_voxels before tightening this.
  // dynamic_min_miss <= 0 turns this off.
  double dynamic_miss_ratio = 2.0;
  int dynamic_min_miss = 3;
  // Rays that would pass more than (floor_band - voxel_size) below their own
  // keyframe's floor stop there, and their returns count as "clipped" rather
  // than hits: specular reflections off a glossy floor, whose rays would
  // otherwise count misses under the floor. A clipped return still counts
  // toward sparse_min_hits, so a one-off reflection goes while a lower level
  // seen from many keyframes (stairs down) stays.
  bool clip_below_floor = true;
  // Also write kOctomapFile at voxel_size; a hit floor voxel stays occupied,
  // since grazing rays would otherwise carve the floor away. The tree is most
  // of the clean's memory; out of memory drops it (octomap_error says so) and
  // map.pcd is still cleaned.
  bool octomap = false;
  // Floor inferred in the tree (not in map.pcd) up to this far (m) from a
  // keyframe on the same level; <= 0 turns it off. The lidar barely samples
  // the floor in the shadows of shelves and walls, and a legged robot's
  // planner needs ground under every step. remover.cpp's fillFloor has the
  // rules.
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
// "hits", "misses", "map", called per keyframe; false stops the clean with
// map.pcd untouched and clean() returns false.
using Progress = std::function<bool(const char *, size_t, size_t)>;

// No poses.txt or no readable patch; its own type so the CLI exits 2, not 3.
class InputError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Replace <dir>/map.pcd (and with Params::octomap write octomap.bt) from
// poses.txt + patches/, via .tmp + rename; .tmp files are removed on failure.
// A save into dir mid-clean changes poses.txt's fingerprint and fails this
// one ("superseded") rather than rename the OLD save's outputs over it.
// Throws InputError or std::runtime_error naming files relative to dir only:
// what() lands in the sidecar, which must hold no absolute path.
bool clean(
  const std::filesystem::path & dir, const Params & params, Measurements & out,
  const Progress & progress = {});

}  // namespace syncai_mapping::dynamic_removal

#endif  // SYNCAI_MAPPING__DYNAMIC_REMOVAL__REMOVER_HPP_

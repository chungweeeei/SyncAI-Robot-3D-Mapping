#ifndef SYNCAI_MAPPING__DYNAMIC_REMOVAL__EDITS_HPP_
#define SYNCAI_MAPPING__DYNAMIC_REMOVAL__EDITS_HPP_

// Hand-patched ground for octomap.bt: the floor voxels an operator painted
// into the console's ground editor (or the standalone viewer), recorded as
// <dir>/octomap_edits.json and applied here. Run by apply_octomap_edits
// (src/apps/apply_octomap_edits.cpp); the README's "Ground patches" has the rules.
//
// The lidar sees the floor only at grazing angles, and the clean's floor fill
// closes only enclosed gaps, so a floor no ray looked into stays unknown and
// the planner refuses it. An operator who knows it is floor marks it; this
// makes the mark the planner's: the voxel occupied and the body clearance
// above it observed-free, which is what octomap_planner's surface rules need
// (no_free_above / clearance_unknown otherwise).
//
// The tree as the clean wrote it is kept as octomap.raw.bt the first time,
// and every apply starts from it, so a patch erased in the console really
// goes and a rerun is idempotent.
//
// No OctoMap or JSON header here: like remover.hpp, so a caller that only
// wants the file names links neither library.

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace syncai_mapping::dynamic_removal::edits
{

// The record: absolute OcTree keys (16 bits an axis, the map origin at
// 32768) of the voxels to make floor, one per xy column, with the tree's
// resolution they were painted on. The backend writes it from the console;
// the standalone viewer's export has the same shape.
inline constexpr const char * kEditsFile = "octomap_edits.json";
// The tree as the clean wrote it, set aside on the first apply. Nothing
// writes it after that; a new clean removes it along with the old tree.
inline constexpr const char * kRawOctomapFile = "octomap.raw.bt";
inline constexpr const char * kEditsFormat = "syncai-octomap-edits";
inline constexpr int kEditsVersion = 1;

struct Params
{
  // The band above a patched floor voxel marked observed-free (m): the
  // planner's body_clearance, which it needs free to stand there. A voxel
  // already occupied in the band is left (a wall foot, a shelf), which the
  // planner then refuses on its own, rightly.
  double clearance = 0.6;
};

struct Edits
{
  double res = 0.0;
  std::vector<std::array<uint16_t, 3>> keys;
};

struct Measurements
{
  size_t patches = 0;  // keys applied
  // What each patched voxel was before: the clean's own floor (occupied), a
  // voxel a ray passed through (free) or one no ray reached (unknown).
  size_t floor_was_occupied = 0;
  size_t floor_was_free = 0;
  size_t floor_was_unknown = 0;
  // The clearance band's voxels: set free here, already free, or left occupied.
  size_t clearance_set_free = 0;
  size_t clearance_already_free = 0;
  size_t clearance_kept_occupied = 0;
  size_t leaves = 0;      // the tree as written
  bool raw_kept = false;  // octomap.raw.bt was written by this run
};

// No tree, no record, or a record this build cannot read; its own type so
// the CLI exits 2, not 3.
class InputError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Parse <dir>/<file>; throws InputError naming the file relative to dir.
Edits read(const std::filesystem::path & dir, const std::string & file = kEditsFile);

// Patch <dir>/octomap.bt from the record: start from octomap.raw.bt (copied
// from octomap.bt if absent), set every key occupied and its clearance band
// free, write octomap.bt.tmp and rename. Throws InputError or
// std::runtime_error naming files relative to dir only.
void apply(
  const std::filesystem::path & dir, const Edits & edits, const Params & params,
  Measurements & out);

}  // namespace syncai_mapping::dynamic_removal::edits

#endif  // SYNCAI_MAPPING__DYNAMIC_REMOVAL__EDITS_HPP_

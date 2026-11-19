#ifndef SYNCAI_MAPPING__DYNAMIC_REMOVAL__RECIPE_HPP_
#define SYNCAI_MAPPING__DYNAMIC_REMOVAL__RECIPE_HPP_

// map_clean.recipe.json, beside map.pcd: the clean's only status surface,
// since pgo_node may be gone (a mode switch) long before the clean ends.
// Modelled on the backend's gridmap.recipe.json:
//
//   {"status":"converting","started_at":"<UTC>","params":{...}}
//   {"status":"converting","started_at":..,"progress":{"stage":"misses","fraction":0.42},"params":{...}}
//   {"status":"ok","started_at":..,"finished_at":..,"params":{...},"measurements":{...}}
//   {"status":"failed","started_at":..,"finished_at":..,"params":{...},"error":"<one line>"}
//
// "progress" is there only once the clean is at work: not in pgo_node's
// document, nor while the clean waits for the host's lock. The backend and
// the console read "fraction" alone (of the whole clean, 0..1); "stage" is
// for whoever cats the file.
//
// "ok" is about map.pcd; octomap.bt is measurements.octomap_written (with
// octomap_error when a requested tree was not), as the tree never fails the
// clean. pgo_node writes "converting" BEFORE spawning clean_map, so a reader
// acting on the save_maps response always finds the file. A hard kill leaves
// "converting" with the raw map.pcd; started_at lets a reader age it out.
// Nothing in it names the map or holds an absolute path.
//
// No OctoMap and no PCL here: this is all pgo_node links of the cleaning.

#include <cstddef>
#include <filesystem>
#include <string>

#include "syncai_mapping/dynamic_removal/remover.hpp"

namespace syncai_mapping::dynamic_removal::recipe
{

inline constexpr const char * kSidecarFile = "map_clean.recipe.json";

// "2026-10-08T03:12:45Z".
std::string isoUtcNow();

// A Progress call as a fraction of the whole clean, 0..1; negative for a stage
// this build does not know. The stages' shares are here, beside the document
// they are written into, so a reader needs no stage names: a new stage is a
// change to this function alone. Never decreasing in the order clean() calls
// it, and under 1 until the clean is over, since "ok" is what says done.
double overallFraction(const std::string & stage, size_t done, size_t total);

std::string converting(const Params & params, const std::string & started_at);
// With "progress": `fraction` as overallFraction answered it.
std::string converting(
  const Params & params, const std::string & started_at, const std::string & stage,
  double fraction);
std::string ok(
  const Params & params, const std::string & started_at, const std::string & finished_at,
  const Measurements & m);
// `error` is flattened to one line.
std::string failed(
  const Params & params, const std::string & started_at, const std::string & finished_at,
  const std::string & error);

// Via .tmp + rename, so a polling reader never sees half a document. Throws
// std::runtime_error (path-free text).
void writeAtomic(const std::filesystem::path & dir, const std::string & json);

}  // namespace syncai_mapping::dynamic_removal::recipe

#endif  // SYNCAI_MAPPING__DYNAMIC_REMOVAL__RECIPE_HPP_

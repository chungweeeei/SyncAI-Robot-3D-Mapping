#ifndef SYNCAI_MAPPING__DYNAMIC_REMOVAL__RECIPE_HPP_
#define SYNCAI_MAPPING__DYNAMIC_REMOVAL__RECIPE_HPP_

// map_clean.recipe.json, beside map.pcd: the clean's only status surface,
// since pgo_node may be gone (a mode switch) long before the clean ends.
// Modelled on the backend's gridmap.recipe.json:
//
//   {"status":"converting","started_at":"<UTC>","params":{...}}
//   {"status":"ok","started_at":..,"finished_at":..,"params":{...},"measurements":{...}}
//   {"status":"failed","started_at":..,"finished_at":..,"params":{...},"error":"<one line>"}
//
// "ok" is about map.pcd; octomap.bt is measurements.octomap_written (with
// octomap_error when a requested tree was not), as the tree never fails the
// clean. pgo_node writes "converting" BEFORE spawning clean_map, so a reader
// acting on the save_maps response always finds the file. A hard kill leaves
// "converting" with the raw map.pcd; started_at lets a reader age it out.
// Nothing in it names the map or holds an absolute path.
//
// No OctoMap and no PCL here: this is all pgo_node links of the cleaning.

#include <filesystem>
#include <string>

#include "syncai_mapping/dynamic_removal/remover.hpp"

namespace syncai_mapping::dynamic_removal::recipe
{

inline constexpr const char * kSidecarFile = "map_clean.recipe.json";

// "2026-10-08T03:12:45Z".
std::string isoUtcNow();

std::string converting(const Params & params, const std::string & started_at);
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

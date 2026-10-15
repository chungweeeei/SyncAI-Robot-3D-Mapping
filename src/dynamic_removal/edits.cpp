#include "syncai_mapping/dynamic_removal/edits.hpp"

#include <nlohmann/json.hpp>
#include <octomap/OcTree.h>

#include <cmath>
#include <fstream>
#include <set>
#include <system_error>
#include <utility>

#include "syncai_mapping/dynamic_removal/remover.hpp"

namespace syncai_mapping::dynamic_removal::edits
{

namespace fs = std::filesystem;

namespace
{

// The key's range; a value outside it is not a key of any tree.
constexpr int kKeyMax = 65535;

fs::path tmpPath(const fs::path & dir, const char * name)
{
  return dir / (std::string(name) + ".tmp");
}

}  // namespace

Edits read(const fs::path & dir, const std::string & file)
{
  const std::string where = file + ": ";
  std::ifstream in(dir / file);
  if (!in) throw InputError(where + "cannot open");
  nlohmann::json doc;
  try {
    in >> doc;
  } catch (const nlohmann::json::exception & e) {
    throw InputError(where + e.what());
  }
  if (!doc.is_object()) throw InputError(where + "not a JSON object");
  if (doc.value("format", "") != kEditsFormat) {
    throw InputError(where + "format is not " + kEditsFormat);
  }
  if (
    !doc.contains("version") || !doc["version"].is_number_integer() ||
    doc["version"].get<int>() != kEditsVersion) {
    throw InputError(where + "version is not " + std::to_string(kEditsVersion));
  }
  if (!doc.contains("res") || !doc["res"].is_number() || doc["res"].get<double>() <= 0.0) {
    throw InputError(where + "res missing or not a positive number");
  }
  if (!doc.contains("fill_key") || !doc["fill_key"].is_array()) {
    throw InputError(where + "fill_key missing or not an array");
  }

  Edits edits;
  edits.res = doc["res"].get<double>();
  std::set<std::pair<uint16_t, uint16_t>> columns;
  size_t i = 0;
  for (const auto & entry : doc["fill_key"]) {
    const std::string at = where + "fill_key[" + std::to_string(i++) + "] ";
    if (!entry.is_array() || entry.size() != 3) throw InputError(at + "is not [kx, ky, kz]");
    std::array<uint16_t, 3> key{};
    for (size_t a = 0; a < 3; ++a) {
      if (!entry[a].is_number_integer()) throw InputError(at + "is not three integers");
      const long long v = entry[a].get<long long>();
      if (v < 0 || v > kKeyMax) throw InputError(at + "is outside 0..65535");
      key[a] = static_cast<uint16_t>(v);
    }
    // One patch per column: the editor fills a column's missing floor, which
    // is one voxel; two at different heights would be a floor over a floor.
    if (!columns.insert({key[0], key[1]}).second) {
      throw InputError(at + "names a column already patched");
    }
    edits.keys.push_back(key);
  }
  return edits;
}

void apply(const fs::path & dir, const Edits & edits, const Params & params, Measurements & out)
{
  const fs::path tree_path = dir / kOctomapFile;
  const fs::path raw_path = dir / kRawOctomapFile;
  std::error_code ec;

  // The clean's tree is the source, once: the first apply sets it aside and
  // every later one starts from it, so the result is a function of the record
  // alone and a patch the operator erased really goes.
  if (!fs::is_regular_file(raw_path)) {
    if (!fs::is_regular_file(tree_path)) {
      throw InputError(std::string("no ") + kOctomapFile + " to patch");
    }
    fs::copy_file(tree_path, raw_path, fs::copy_options::none, ec);
    if (ec) {
      throw std::runtime_error(std::string("cannot keep ") + kRawOctomapFile + ": " + ec.message());
    }
    out.raw_kept = true;
  }

  // The resolution is read off the file by readBinary; the constructor's is
  // overwritten. Checked against the record's: keys index cells, so a record
  // painted at another resolution lands in the wrong places.
  octomap::OcTree tree(0.1);
  if (!tree.readBinary(raw_path.string())) {
    throw InputError(std::string("cannot read ") + kRawOctomapFile);
  }
  if (std::fabs(tree.getResolution() - edits.res) > 1e-9) {
    throw InputError(
      std::string(kEditsFile) + ": res " + std::to_string(edits.res) + " but the tree is " +
      std::to_string(tree.getResolution()));
  }

  const float occupied_log = tree.getClampingThresMaxLog();
  const float free_log = tree.getClampingThresMinLog();
  const int clearance_cells =
    std::max(1, static_cast<int>(std::ceil(params.clearance / tree.getResolution() - 1e-9)));

  for (const auto & k : edits.keys) {
    const octomap::OcTreeKey key(k[0], k[1], k[2]);
    if (const auto * node = tree.search(key)) {
      (tree.isNodeOccupied(node) ? out.floor_was_occupied : out.floor_was_free)++;
    } else {
      ++out.floor_was_unknown;
    }
    tree.setNodeValue(key, occupied_log, true);
    for (int dz = 1; dz <= clearance_cells; ++dz) {
      const octomap::OcTreeKey above(k[0], k[1], static_cast<uint16_t>(k[2] + dz));
      if (const auto * node = tree.search(above)) {
        (tree.isNodeOccupied(node) ? out.clearance_kept_occupied : out.clearance_already_free)++;
        continue;
      }
      tree.setNodeValue(above, free_log, true);
      ++out.clearance_set_free;
    }
    ++out.patches;
  }
  tree.updateInnerOccupancy();

  // Like the clean: .tmp + rename, so a reader never sees a torn tree, and
  // the old one stays if the write fails.
  const fs::path tmp = tmpPath(dir, kOctomapFile);
  if (!tree.writeBinary(tmp.string())) {
    fs::remove(tmp, ec);
    throw std::runtime_error(std::string("cannot write ") + kOctomapFile);
  }
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) ++out.leaves;
  fs::rename(tmp, tree_path, ec);
  if (ec) {
    fs::remove(tmp, ec);
    throw std::runtime_error(std::string("cannot rename ") + kOctomapFile + ": " + ec.message());
  }
}

}  // namespace syncai_mapping::dynamic_removal::edits

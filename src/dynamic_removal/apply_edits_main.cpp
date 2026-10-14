// apply_octomap_edits <map_dir> [--clearance 0.6] [--edits octomap_edits.json]:
// patch a saved map's octomap.bt with the ground the console's editor
// marked. Run by hand after a save in the console:
// `ros2 run syncai_mapping apply_octomap_edits map/<name>`.
//
// Exit codes: 0 ok, 1 usage, 2 no usable input (no octomap.bt, no record, a
// record this build cannot read), 3 apply or write failed.
//
// Not a ROS node, for clean_map's reason: a batch job needs no DDS
// participant, and it then runs anywhere with OctoMap.

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "syncai_mapping/dynamic_removal/edits.hpp"

namespace
{

namespace fs = std::filesystem;
namespace ed = syncai_mapping::dynamic_removal::edits;

void usage()
{
  std::cerr << "usage: apply_octomap_edits <map_dir> [--clearance 0.6] [--edits " << ed::kEditsFile
            << "]\n";
}

bool parseDouble(const std::string & s, double & out)
{
  try {
    size_t used = 0;
    out = std::stod(s, &used);
    return used == s.size();
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string dir_arg, edits_file = ed::kEditsFile;
  ed::Params params;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      usage();
      return 0;
    }
    if (a.rfind("--", 0) != 0) {
      if (!dir_arg.empty()) {
        usage();
        return 1;
      }
      dir_arg = a;
      continue;
    }
    if (i + 1 >= argc) {
      usage();
      return 1;
    }
    const std::string v = argv[++i];
    bool good = true;
    if (a == "--clearance") {
      good = parseDouble(v, params.clearance) && params.clearance >= 0.0;
    } else if (a == "--edits") {
      // A bare name inside the map directory, like every file there.
      good = !v.empty() && fs::path(v).filename() == fs::path(v);
      edits_file = v;
    } else {
      good = false;
    }
    if (!good) {
      std::cerr << "[apply_octomap_edits] bad option " << a << " " << v << "\n";
      usage();
      return 1;
    }
  }
  if (dir_arg.empty()) {
    usage();
    return 1;
  }
  const fs::path dir(dir_arg);
  if (!fs::is_directory(dir)) {
    std::cerr << "[apply_octomap_edits] " << dir << " is not a directory\n";
    return 2;
  }

  ed::Measurements m;
  try {
    const ed::Edits edits = ed::read(dir, edits_file);
    std::cout << "[apply_octomap_edits] " << edits.keys.size() << " ground patches at " << edits.res
              << " m in " << dir.string() << std::endl;
    ed::apply(dir, edits, params, m);
  } catch (const ed::InputError & e) {
    std::cerr << "[apply_octomap_edits] " << e.what() << "\n";
    return 2;
  } catch (const std::exception & e) {
    std::cerr << "[apply_octomap_edits] FAILED: " << e.what() << "\n";
    return 3;
  }
  std::cout << "[apply_octomap_edits] DONE patches=" << m.patches
            << " (were occupied=" << m.floor_was_occupied << " free=" << m.floor_was_free
            << " unknown=" << m.floor_was_unknown << ") clearance_set_free=" << m.clearance_set_free
            << " already_free=" << m.clearance_already_free
            << " kept_occupied=" << m.clearance_kept_occupied << " leaves=" << m.leaves
            << (m.raw_kept ? std::string(" kept ") + ed::kRawOctomapFile : "") << std::endl;
  return 0;
}

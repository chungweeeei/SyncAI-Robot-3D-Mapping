#include <gtest/gtest.h>
#include <octomap/OcTree.h>
#include <unistd.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "syncai_mapping/dynamic_removal/edits.hpp"
#include "syncai_mapping/dynamic_removal/remover.hpp"

namespace
{

namespace fs = std::filesystem;
namespace ed = syncai_mapping::dynamic_removal::edits;
using syncai_mapping::dynamic_removal::kOctomapFile;

constexpr double kRes = 0.1;
// ceil(0.6 / 0.1): the default clearance band in cells.
constexpr int kClearanceCells = 6;

// A fresh map directory per test, removed afterwards.
class OctomapEditsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    dir_ = fs::temp_directory_path() /
           ("syncai_octomap_edits_" + std::to_string(::getpid()) + "_" + info->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_);
  }

  void TearDown() override { fs::remove_all(dir_); }

  void writeFile(const std::string & name, const std::string & text) const
  {
    std::ofstream(dir_ / name) << text;
  }

  // A record in the console's shape, with the given fill_key array.
  void writeEdits(const std::string & fill_key, double res = kRes) const
  {
    writeFile(
      ed::kEditsFile, R"({"format": "syncai-octomap-edits", "version": 1, "res": )" +
                        std::to_string(res) + R"(, "fill_key": )" + fill_key + "}");
  }

  // The clean's tree as a stand-in: one occupied and one free voxel, at the
  // keys the tests patch over.
  void writeTree(double res = kRes) const
  {
    octomap::OcTree tree(res);
    tree.setNodeValue(occupied_, tree.getClampingThresMaxLog(), true);
    tree.setNodeValue(free_, tree.getClampingThresMinLog(), true);
    tree.updateInnerOccupancy();
    ASSERT_TRUE(tree.writeBinary((dir_ / kOctomapFile).string()));
  }

  octomap::OcTree readTree(const std::string & name = kOctomapFile) const
  {
    octomap::OcTree tree(kRes);
    EXPECT_TRUE(tree.readBinary((dir_ / name).string()));
    return tree;
  }

  std::string readBytes(const std::string & name) const
  {
    std::ifstream in(dir_ / name, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
  }

  // Keys around the map origin, each in its own column.
  const octomap::OcTreeKey occupied_{32768, 32768, 32768};
  const octomap::OcTreeKey free_{32770, 32768, 32768};
  const octomap::OcTreeKey unknown_{32772, 32768, 32768};

  fs::path dir_;
};

enum class State { kUnknown, kFree, kOccupied };

State stateAt(const octomap::OcTree & tree, const octomap::OcTreeKey & key)
{
  const auto * node = tree.search(key);
  if (!node) return State::kUnknown;
  return tree.isNodeOccupied(node) ? State::kOccupied : State::kFree;
}

octomap::OcTreeKey above(const octomap::OcTreeKey & key, int dz)
{
  return octomap::OcTreeKey(key[0], key[1], static_cast<uint16_t>(key[2] + dz));
}

// ---- read ---------------------------------------------------------------------

TEST_F(OctomapEditsTest, ReadParsesTheRecord)
{
  writeEdits("[[32768, 32768, 32760], [0, 65535, 1]]", 0.05);
  const ed::Edits edits = ed::read(dir_);
  EXPECT_DOUBLE_EQ(edits.res, 0.05);
  ASSERT_EQ(edits.keys.size(), 2u);
  EXPECT_EQ(edits.keys[0], (std::array<uint16_t, 3>{32768, 32768, 32760}));
  EXPECT_EQ(edits.keys[1], (std::array<uint16_t, 3>{0, 65535, 1}));
}

TEST_F(OctomapEditsTest, ReadAcceptsAnEmptyRecord)
{
  writeEdits("[]");
  EXPECT_TRUE(ed::read(dir_).keys.empty());
}

TEST_F(OctomapEditsTest, ReadTakesAnotherFileName)
{
  writeFile("other.json", R"({"format": "syncai-octomap-edits", "version": 1, "res": 0.1,
                      "fill_key": [[1, 2, 3]]})");
  EXPECT_EQ(ed::read(dir_, "other.json").keys.size(), 1u);
}

TEST_F(OctomapEditsTest, ReadRejectsAMissingFile)
{
  EXPECT_THROW(ed::read(dir_), ed::InputError);
}

TEST_F(OctomapEditsTest, ReadRejectsMalformedRecords)
{
  const char * bad[] = {
    "not json",
    "[]",
    R"({"format": "other", "version": 1, "res": 0.1, "fill_key": []})",
    R"({"version": 1, "res": 0.1, "fill_key": []})",
    R"({"format": "syncai-octomap-edits", "version": 2, "res": 0.1, "fill_key": []})",
    R"({"format": "syncai-octomap-edits", "version": "1", "res": 0.1, "fill_key": []})",
    R"({"format": "syncai-octomap-edits", "res": 0.1, "fill_key": []})",
    R"({"format": "syncai-octomap-edits", "version": 1, "res": 0, "fill_key": []})",
    R"({"format": "syncai-octomap-edits", "version": 1, "res": -0.1, "fill_key": []})",
    R"({"format": "syncai-octomap-edits", "version": 1, "fill_key": []})",
    R"({"format": "syncai-octomap-edits", "version": 1, "res": 0.1})",
    R"({"format": "syncai-octomap-edits", "version": 1, "res": 0.1, "fill_key": {}})",
  };
  for (const char * text : bad) {
    writeFile(ed::kEditsFile, text);
    EXPECT_THROW(ed::read(dir_), ed::InputError) << text;
  }
}

TEST_F(OctomapEditsTest, ReadRejectsMalformedKeys)
{
  const char * bad[] = {
    "[[1, 2]]",
    "[[1, 2, 3, 4]]",
    "[1]",
    "[[1, 2, 3.5]]",
    R"([[1, 2, "3"]])",
    "[[-1, 2, 3]]",
    "[[1, 65536, 3]]",
    // Two patches in one column, even at different heights.
    "[[1, 2, 3], [1, 2, 9]]",
  };
  for (const char * keys : bad) {
    writeEdits(keys);
    EXPECT_THROW(ed::read(dir_), ed::InputError) << keys;
  }
}

// ---- apply --------------------------------------------------------------------

TEST_F(OctomapEditsTest, ApplyNeedsATree)
{
  ed::Measurements m;
  EXPECT_THROW(ed::apply(dir_, ed::Edits{kRes, {}}, ed::Params{}, m), ed::InputError);
  EXPECT_FALSE(fs::exists(dir_ / ed::kRawOctomapFile));
}

TEST_F(OctomapEditsTest, ApplyRejectsAnotherResolution)
{
  writeTree();
  const std::string before = readBytes(kOctomapFile);
  ed::Measurements m;
  EXPECT_THROW(ed::apply(dir_, ed::Edits{0.05, {}}, ed::Params{}, m), ed::InputError);
  EXPECT_EQ(readBytes(kOctomapFile), before);
}

TEST_F(OctomapEditsTest, ApplyMakesFloorWithFreeSpaceAbove)
{
  writeTree();
  const std::string clean = readBytes(kOctomapFile);
  const ed::Edits edits{
    kRes,
    {{occupied_[0], occupied_[1], occupied_[2]},
     {free_[0], free_[1], free_[2]},
     {unknown_[0], unknown_[1], unknown_[2]}}};
  ed::Measurements m;
  ed::apply(dir_, edits, ed::Params{}, m);

  EXPECT_EQ(m.patches, 3u);
  EXPECT_EQ(m.floor_was_occupied, 1u);
  EXPECT_EQ(m.floor_was_free, 1u);
  EXPECT_EQ(m.floor_was_unknown, 1u);
  EXPECT_EQ(m.clearance_set_free, 3u * kClearanceCells);
  EXPECT_EQ(m.clearance_already_free, 0u);
  EXPECT_EQ(m.clearance_kept_occupied, 0u);
  EXPECT_GT(m.leaves, 0u);
  EXPECT_TRUE(m.raw_kept);

  // The clean's tree is set aside untouched, and no .tmp is left.
  EXPECT_EQ(readBytes(ed::kRawOctomapFile), clean);
  EXPECT_FALSE(fs::exists(dir_ / (std::string(kOctomapFile) + ".tmp")));

  const octomap::OcTree tree = readTree();
  for (const auto & key : {occupied_, free_, unknown_}) {
    EXPECT_EQ(stateAt(tree, key), State::kOccupied);
    for (int dz = 1; dz <= kClearanceCells; ++dz) {
      EXPECT_EQ(stateAt(tree, above(key, dz)), State::kFree) << "dz " << dz;
    }
    // The band stops at the clearance.
    EXPECT_EQ(stateAt(tree, above(key, kClearanceCells + 1)), State::kUnknown);
  }
}

TEST_F(OctomapEditsTest, ApplyLeavesOccupiedVoxelsInTheBand)
{
  // A shelf two cells over the patch, and a voxel a ray passed through.
  {
    octomap::OcTree tree(kRes);
    tree.setNodeValue(above(unknown_, 2), tree.getClampingThresMaxLog(), true);
    tree.setNodeValue(above(unknown_, 4), tree.getClampingThresMinLog(), true);
    tree.updateInnerOccupancy();
    ASSERT_TRUE(tree.writeBinary((dir_ / kOctomapFile).string()));
  }
  ed::Measurements m;
  ed::apply(dir_, ed::Edits{kRes, {{unknown_[0], unknown_[1], unknown_[2]}}}, ed::Params{}, m);

  EXPECT_EQ(m.clearance_kept_occupied, 1u);
  EXPECT_EQ(m.clearance_already_free, 1u);
  EXPECT_EQ(m.clearance_set_free, kClearanceCells - 2u);
  const octomap::OcTree tree = readTree();
  EXPECT_EQ(stateAt(tree, above(unknown_, 2)), State::kOccupied);
  EXPECT_EQ(stateAt(tree, above(unknown_, 4)), State::kFree);
}

TEST_F(OctomapEditsTest, ApplyClearsAtLeastOneCell)
{
  writeTree();
  ed::Params params;
  params.clearance = 0.0;
  ed::Measurements m;
  ed::apply(dir_, ed::Edits{kRes, {{unknown_[0], unknown_[1], unknown_[2]}}}, params, m);

  EXPECT_EQ(m.clearance_set_free, 1u);
  const octomap::OcTree tree = readTree();
  EXPECT_EQ(stateAt(tree, above(unknown_, 1)), State::kFree);
  EXPECT_EQ(stateAt(tree, above(unknown_, 2)), State::kUnknown);
}

TEST_F(OctomapEditsTest, ApplyIsIdempotent)
{
  writeTree();
  const ed::Edits edits{kRes, {{unknown_[0], unknown_[1], unknown_[2]}}};
  ed::Measurements first, second;
  ed::apply(dir_, edits, ed::Params{}, first);
  const std::string once = readBytes(kOctomapFile);
  const std::string raw = readBytes(ed::kRawOctomapFile);
  ed::apply(dir_, edits, ed::Params{}, second);

  // The second run starts from the raw copy again, so it sees the same tree.
  EXPECT_FALSE(second.raw_kept);
  EXPECT_EQ(second.floor_was_unknown, 1u);
  EXPECT_EQ(second.clearance_set_free, first.clearance_set_free);
  EXPECT_EQ(readBytes(kOctomapFile), once);
  EXPECT_EQ(readBytes(ed::kRawOctomapFile), raw);
}

TEST_F(OctomapEditsTest, ApplyDropsAnErasedPatch)
{
  writeTree();
  ed::Measurements m;
  ed::apply(dir_, ed::Edits{kRes, {{unknown_[0], unknown_[1], unknown_[2]}}}, ed::Params{}, m);
  ASSERT_EQ(stateAt(readTree(), unknown_), State::kOccupied);

  // The operator erased it in the console: the record is empty now.
  ed::apply(dir_, ed::Edits{kRes, {}}, ed::Params{}, m);
  const octomap::OcTree tree = readTree();
  EXPECT_EQ(stateAt(tree, unknown_), State::kUnknown);
  EXPECT_EQ(stateAt(tree, above(unknown_, 1)), State::kUnknown);
  EXPECT_EQ(stateAt(tree, occupied_), State::kOccupied);
  EXPECT_EQ(stateAt(tree, free_), State::kFree);
}

TEST_F(OctomapEditsTest, ApplyStartsFromTheRawCopy)
{
  // A raw copy present means octomap.bt is a patched tree; it is not reread.
  writeTree();
  fs::copy_file(dir_ / kOctomapFile, dir_ / ed::kRawOctomapFile);
  writeFile(kOctomapFile, "a previous apply's output");
  ed::Measurements m;
  ed::apply(dir_, ed::Edits{kRes, {}}, ed::Params{}, m);

  EXPECT_FALSE(m.raw_kept);
  EXPECT_EQ(readBytes(kOctomapFile), readBytes(ed::kRawOctomapFile));
}

}  // namespace

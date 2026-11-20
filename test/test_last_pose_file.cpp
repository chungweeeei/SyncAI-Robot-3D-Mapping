// The localizer's last pose file as resume_mapping reads it: a whole file is a hint, anything
// less is none.
#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "syncai_mapping/pgos/last_pose_file.h"

namespace
{
namespace fs = std::filesystem;

class LastPoseFileTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    m_path = fs::temp_directory_path() / ("test_last_pose_" + std::to_string(getpid()) + ".json");
    fs::remove(m_path);
  }
  void TearDown() override { fs::remove(m_path); }
  void write(const std::string & text) { std::ofstream(m_path, std::ios::trunc) << text; }
  fs::path m_path;
};

// As syncai_localizer's saveLastPose writes it (ordered keys, two-space indent).
const char * kWhole = R"({
  "map": "dp",
  "x": -8.87306380086245,
  "y": -10.178577163936087,
  "ground_z": 1.007487871279471,
  "yaw": -1.5547339771475572,
  "saved_at": "2026-11-14T02:37:06Z"
}
)";
}  // namespace

TEST_F(LastPoseFileTest, ReadsWhatTheLocalizerWrites)
{
  write(kWhole);
  std::string why;
  const auto pose = readLastPoseFile(m_path, why);
  ASSERT_TRUE(pose) << why;
  EXPECT_EQ(pose->map, "dp");
  EXPECT_DOUBLE_EQ(pose->x, -8.87306380086245);
  EXPECT_DOUBLE_EQ(pose->y, -10.178577163936087);
  EXPECT_DOUBLE_EQ(pose->ground_z, 1.007487871279471);
  EXPECT_DOUBLE_EQ(pose->yaw, -1.5547339771475572);
  EXPECT_EQ(pose->saved_at, "2026-11-14T02:37:06Z");
}

TEST_F(LastPoseFileTest, AMissingFileIsNoHint)
{
  std::string why;
  EXPECT_FALSE(readLastPoseFile(m_path, why));
  EXPECT_FALSE(why.empty());
}

// Cut anywhere, the file is not a pose: half a number would be a hint metres off.
TEST_F(LastPoseFileTest, ATruncatedFileIsNoHint)
{
  const std::string whole = kWhole;
  for (size_t n : {size_t(0), size_t(1), whole.size() / 3, whole.size() / 2, whole.size() - 4}) {
    write(whole.substr(0, n));
    std::string why;
    EXPECT_FALSE(readLastPoseFile(m_path, why)) << "cut at " << n;
  }
}

TEST_F(LastPoseFileTest, AMissingOrMistypedFieldIsNoHint)
{
  for (const char * text : {
         R"({"x": 1.0, "y": 2.0, "ground_z": 0.0, "yaw": 0.1})",
         R"({"map": "", "x": 1.0, "y": 2.0, "ground_z": 0.0, "yaw": 0.1})",
         R"({"map": "dp", "y": 2.0, "ground_z": 0.0, "yaw": 0.1})",
         R"({"map": "dp", "x": "1.0", "y": 2.0, "ground_z": 0.0, "yaw": 0.1})",
         R"({"map": "dp", "x": 1.0, "y": 2.0, "yaw": 0.1})",
         R"({"map": "dp", "x": 1.0, "y": 2.0, "ground_z": 0.0})",
         R"([1, 2, 3])",
       }) {
    write(text);
    std::string why;
    EXPECT_FALSE(readLastPoseFile(m_path, why)) << text;
    EXPECT_FALSE(why.empty()) << text;
  }
}

// saved_at is for whoever reads the file; a pose without it is still a pose.
TEST_F(LastPoseFileTest, SavedAtIsOptional)
{
  write(R"({"map": "dp", "x": 1.0, "y": 2.0, "ground_z": 0.5, "yaw": 0.1})");
  std::string why;
  const auto pose = readLastPoseFile(m_path, why);
  ASSERT_TRUE(pose) << why;
  EXPECT_TRUE(pose->saved_at.empty());
}

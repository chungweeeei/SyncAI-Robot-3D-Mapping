// A map directory written by writeKeyframes and read back by readSavedMap: what resume_mapping
// loads must be what save_maps wrote, and a directory that is not a whole map must be refused
// rather than resumed short.
#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <string>
#include <vector>

#include "syncai_mapping/pgos/map_reader.h"
#include "syncai_mapping/pgos/map_writer.h"

namespace
{
namespace fs = std::filesystem;

std::vector<KeyPoseWithCloud> randomKeyPoses(size_t n, std::mt19937 & rng)
{
  std::uniform_real_distribution<float> coord(-20.0f, 20.0f), inten(0.0f, 255.0f);
  std::uniform_real_distribution<double> angle(-M_PI, M_PI), tilt(-0.1, 0.1), pos(-50.0, 50.0);
  std::uniform_int_distribution<int> count(1, 500);
  std::vector<KeyPoseWithCloud> ret(n);
  for (KeyPoseWithCloud & kp : ret) {
    kp.body_cloud.reset(new CloudType);
    const int m = count(rng);
    for (int i = 0; i < m; i++) {
      PointType p;
      p.x = coord(rng);
      p.y = coord(rng);
      p.z = coord(rng);
      p.intensity = inten(rng);
      kp.body_cloud->push_back(p);
    }
    kp.r_global =
      (Eigen::AngleAxisd(angle(rng), V3D::UnitZ()) * Eigen::AngleAxisd(tilt(rng), V3D::UnitY()) *
       Eigen::AngleAxisd(tilt(rng), V3D::UnitX()))
        .toRotationMatrix();
    kp.t_global = V3D(pos(rng), pos(rng), tilt(rng));
  }
  return ret;
}

std::string readAll(const fs::path & path)
{
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void writeText(const fs::path & path, const std::string & text)
{
  std::ofstream out(path, std::ios::trunc);
  out << text;
}

std::string errorOf(const fs::path & dir)
{
  try {
    readSavedMap(dir);
  } catch (const std::runtime_error & e) {
    return e.what();
  }
  return "";
}

class MapReaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    m_dir = fs::temp_directory_path() / ("test_map_reader_" + std::to_string(getpid()));
    fs::remove_all(m_dir);
    fs::create_directories(m_dir);
  }
  void TearDown() override { fs::remove_all(m_dir); }
  fs::path m_dir;
};
}  // namespace

TEST_F(MapReaderTest, ReadsBackWhatWasWritten)
{
  std::mt19937 rng(3);
  const std::vector<KeyPoseWithCloud> written = randomKeyPoses(12, rng);
  writeKeyframes(m_dir, written, {0});
  EXPECT_FALSE(fs::exists(m_dir / kSessionsFile)) << "one run has no sessions.json";

  const SavedMap map = readSavedMap(m_dir);
  EXPECT_TRUE(map.warning.empty());
  EXPECT_EQ(map.session_starts, std::vector<size_t>{0});
  ASSERT_EQ(map.key_poses.size(), written.size());
  for (size_t i = 0; i < written.size(); i++) {
    // poses.txt carries 6 significant digits.
    EXPECT_LT((map.key_poses[i].t_global - written[i].t_global).norm(), 1e-3);
    EXPECT_LT(
      Eigen::Quaterniond(map.key_poses[i].r_global)
        .angularDistance(Eigen::Quaterniond(written[i].r_global)),
      1e-4);
    ASSERT_EQ(map.key_poses[i].body_cloud->size(), written[i].body_cloud->size());
    for (size_t k = 0; k < written[i].body_cloud->size(); k++) {
      const PointType & a = map.key_poses[i].body_cloud->points[k];
      const PointType & b = written[i].body_cloud->points[k];
      EXPECT_TRUE(a.x == b.x && a.y == b.y && a.z == b.z && a.intensity == b.intensity);
    }
  }
}

// What "frozen" means on disk: a map resumed and saved again, with nothing added, lists the same
// poses. The translations to the digit; a quaternion goes through a rotation matrix on the way.
TEST_F(MapReaderTest, PosesSurviveARoundTrip)
{
  std::mt19937 rng(5);
  writeKeyframes(m_dir, randomKeyPoses(30, rng), {0});
  const SavedMap first = readSavedMap(m_dir);

  const fs::path again = m_dir / "again";
  fs::create_directories(again);
  writeKeyframes(again, first.key_poses, first.session_starts);
  const SavedMap second = readSavedMap(again);

  ASSERT_EQ(second.key_poses.size(), first.key_poses.size());
  for (size_t i = 0; i < first.key_poses.size(); i++) {
    EXPECT_TRUE(second.key_poses[i].t_global == first.key_poses[i].t_global) << "keyframe " << i;
    EXPECT_LT(
      Eigen::Quaterniond(second.key_poses[i].r_global)
        .angularDistance(Eigen::Quaterniond(first.key_poses[i].r_global)),
      1e-5);
  }
}

TEST_F(MapReaderTest, SessionsFollowThePoses)
{
  std::mt19937 rng(7);
  const std::vector<KeyPoseWithCloud> key_poses = randomKeyPoses(10, rng);

  // Two runs, and a third that banked nothing: left out.
  writeKeyframes(m_dir, key_poses, {0, 6, 10});
  ASSERT_TRUE(fs::exists(m_dir / kSessionsFile));
  EXPECT_FALSE(fs::exists(m_dir / (std::string(kSessionsFile) + ".tmp")));
  SavedMap map = readSavedMap(m_dir);
  EXPECT_TRUE(map.warning.empty());
  EXPECT_EQ(map.session_starts, (std::vector<size_t>{0, 6}));
  // Neither a name nor a path: a map directory can be renamed.
  EXPECT_EQ(readAll(m_dir / kSessionsFile).find(m_dir.filename().string()), std::string::npos);

  // A save of one run into the same directory takes the stale file with it.
  writeKeyframes(m_dir, key_poses, {0});
  EXPECT_FALSE(fs::exists(m_dir / kSessionsFile));
  EXPECT_EQ(readSavedMap(m_dir).session_starts, std::vector<size_t>{0});
}

// A sidecar that does not tile the poses is dropped whole, with a warning, never half used.
TEST_F(MapReaderTest, UnusableSessionsAreIgnored)
{
  std::mt19937 rng(9);
  writeKeyframes(m_dir, randomKeyPoses(10, rng), {0});
  const char * bad[] = {
    "not json",
    R"({"version": 2, "sessions": [{"first": 0, "count": 10}]})",
    R"({"version": 1, "sessions": []})",
    R"({"version": 1, "sessions": [{"first": 0, "count": 4}, {"first": 5, "count": 5}]})",
    R"({"version": 1, "sessions": [{"first": 0, "count": 4}, {"first": 4, "count": 5}]})",
    R"({"version": 1, "sessions": [{"first": 0, "count": 4}, {"first": 4, "count": 9}]})",
    R"({"version": 1, "sessions": [{"first": 0}]})",
  };
  for (const char * text : bad) {
    writeText(m_dir / kSessionsFile, text);
    const SavedMap map = readSavedMap(m_dir);
    EXPECT_FALSE(map.warning.empty()) << text;
    EXPECT_EQ(map.session_starts, std::vector<size_t>{0}) << text;
    EXPECT_EQ(map.key_poses.size(), 10u) << text;
  }
}

TEST_F(MapReaderTest, RefusesWhatIsNotAWholeMap)
{
  EXPECT_NE(errorOf(m_dir / "missing"), "");
  // Saved without patches: map.pcd alone.
  EXPECT_NE(errorOf(m_dir).find("poses.txt"), std::string::npos);

  std::mt19937 rng(11);
  writeKeyframes(m_dir, randomKeyPoses(5, rng), {0});
  const std::string poses = readAll(m_dir / "poses.txt");
  ASSERT_NO_THROW(readSavedMap(m_dir));

  fs::remove(m_dir / "patches" / "3.pcd");
  EXPECT_NE(errorOf(m_dir).find("patches/3.pcd"), std::string::npos);

  writeKeyframes(m_dir, randomKeyPoses(5, rng), {0});
  writeText(m_dir / "poses.txt", poses + "5.pcd 1 2 three 1 0 0 0\n");
  EXPECT_NE(errorOf(m_dir).find("line 6"), std::string::npos);

  writeText(m_dir / "poses.txt", "../patches/0.pcd 1 2 3 1 0 0 0\n");
  EXPECT_NE(errorOf(m_dir).find("line 1"), std::string::npos);

  writeText(m_dir / "poses.txt", "0.pcd 1 2 3 0 0 0 0\n");
  EXPECT_NE(errorOf(m_dir).find("rotation"), std::string::npos);

  writeText(m_dir / "poses.txt", "\n\n");
  EXPECT_NE(errorOf(m_dir).find("no keyframes"), std::string::npos);
}

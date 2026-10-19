// writeMergedMapPcd against what saveMapsCB used to do: merge every keyframe in memory, then
// pcl::io::savePCDFileBinary. map.pcd is read by clean_map and the backend's gridmap recipes, so
// the streamed file must be the same bytes, not just the same points.
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <string>
#include <vector>

#include "syncai_mapping/pgos/map_writer.h"

namespace
{
namespace fs = std::filesystem;

std::vector<KeyPoseWithCloud> randomKeyPoses(size_t n, std::mt19937 & rng)
{
  std::uniform_real_distribution<float> coord(-20.0f, 20.0f), inten(0.0f, 255.0f);
  std::uniform_real_distribution<double> angle(-M_PI, M_PI), tilt(-0.1, 0.1), pos(-50.0, 50.0);
  std::uniform_int_distribution<int> count(0, 3000);
  std::vector<KeyPoseWithCloud> ret(n);
  for (KeyPoseWithCloud & kp : ret) {
    kp.body_cloud.reset(new CloudType);
    const int m = count(rng);  // 0 included: an empty keyframe must not break the packing
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

// saveMapsCB before writeMergedMapPcd.
void writeInMemory(const fs::path & path, const std::vector<KeyPoseWithCloud> & key_poses)
{
  CloudType::Ptr ret(new CloudType);
  for (const KeyPoseWithCloud & kp : key_poses) {
    CloudType::Ptr world_cloud(new CloudType);
    pcl::transformPointCloud(
      *kp.body_cloud, *world_cloud, kp.t_global, Eigen::Quaterniond(kp.r_global));
    *ret += *world_cloud;
  }
  pcl::io::savePCDFileBinary(path.string(), *ret);
}

std::string readAll(const fs::path & path)
{
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

class MapWriterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    m_dir = fs::temp_directory_path() / ("test_map_writer_" + std::to_string(getpid()));
    fs::create_directories(m_dir);
  }
  void TearDown() override { fs::remove_all(m_dir); }
  fs::path m_dir;
};
}  // namespace

TEST_F(MapWriterTest, SameBytesAsInMemoryMerge)
{
  std::mt19937 rng(3);
  const std::vector<KeyPoseWithCloud> key_poses = randomKeyPoses(40, rng);
  writeInMemory(m_dir / "expected.pcd", key_poses);
  writeMergedMapPcd(m_dir / "map.pcd", key_poses);

  const std::string expected = readAll(m_dir / "expected.pcd");
  const std::string actual = readAll(m_dir / "map.pcd");
  ASSERT_FALSE(expected.empty());
  EXPECT_EQ(actual.size(), expected.size());
  EXPECT_TRUE(actual == expected) << "streamed map.pcd differs from the in-memory merge";
  EXPECT_FALSE(fs::exists(m_dir / "map.pcd.save.tmp"));

  CloudType loaded;
  ASSERT_EQ(pcl::io::loadPCDFile((m_dir / "map.pcd").string(), loaded), 0);
  size_t total = 0;
  for (const KeyPoseWithCloud & kp : key_poses) total += kp.body_cloud->size();
  EXPECT_EQ(loaded.size(), total);
}

// A rewrite replaces the previous file whole, and a failure leaves it alone.
TEST_F(MapWriterTest, ReplacesAtomically)
{
  std::mt19937 rng(5);
  const fs::path map = m_dir / "map.pcd";
  writeMergedMapPcd(map, randomKeyPoses(5, rng));
  const std::vector<KeyPoseWithCloud> second = randomKeyPoses(3, rng);
  writeMergedMapPcd(map, second);
  writeInMemory(m_dir / "expected.pcd", second);
  EXPECT_TRUE(readAll(map) == readAll(m_dir / "expected.pcd"));

  const std::string before = readAll(map);
  EXPECT_THROW(writeMergedMapPcd(m_dir / "missing_dir" / "map.pcd", second), std::runtime_error);
  EXPECT_TRUE(readAll(map) == before);
}

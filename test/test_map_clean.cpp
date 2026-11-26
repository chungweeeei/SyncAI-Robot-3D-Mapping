// The cleaner on a generated save directory: one room, a robot walking its
// length, and a 1 m crate that is there for some keyframes and not for others.
// What the clean makes of the crate depends on WHICH keyframes saw it, and for
// a resumed map on which run they belong to (sessions.json):
//
//   there from some run on        -> kept, occupied in the tree
//   there in an earlier run only  -> removed, free
//   there for two scans of a run  -> a person: removed, in either run
//
// The first is what sessions.json is read for. Counted over all keyframes
// alike, a crate that arrived between two runs has a person's signature (hit
// by the new run's few scans, seen through by every old one) and went from
// map.pcd and from the planner's tree.
//
// And a blocker ON the robot's path, there only after the robot walked through
// it, for many scans: the operator standing at a stair foot the robot had just
// left (dp_1125). Seen standing about as often as seen through, it passes the
// dynamic rule; the path rule (track_clear_radius) takes it.
#include <gtest/gtest.h>
#include <octomap/OcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "syncai_mapping/dynamic_removal/remover.hpp"
#include "syncai_mapping/pgos/sessions.h"

namespace
{

namespace fs = std::filesystem;
namespace dr = syncai_mapping::dynamic_removal;

// Every surface sits 3 cm off the 0.1 m voxel grid, so no return lands on a
// voxel boundary and its voxel is the same in every scan.
constexpr double kOff = 0.03;
constexpr double kRoomX = 8.0, kRoomY = 6.0, kRoomZ = 2.6;
constexpr double kLidarHeight = 0.481;  // Params::lidar_height
// The robot walks along y = kPathY; the crate stands beside the path, between
// it and the far wall, so a scan taken without it looks through where it is.
constexpr double kPathY = kOff + 2.0;
constexpr double kCrateX0 = kOff + 3.5, kCrateX1 = kOff + 4.5;
constexpr double kCrateY0 = kOff + 4.0, kCrateY1 = kOff + 5.0;
constexpr double kCrateZ1 = kOff + 1.0;
// The part of the crate the dynamic rule judges: clear of the floor band.
constexpr double kJudgedZ0 = kOff + 0.3, kJudgedZ1 = kOff + 0.9;
// The blocker: 0.4 m square on the path, 1 m high. Judged within 0.15 m of the
// path (the rule's radius is 0.2) and inside the rule's band, floor + 0.25 to
// floor + track_clear_height (0.6).
constexpr double kBlockX0 = kOff + 5.0, kBlockX1 = kOff + 5.4;
constexpr double kBlockY0 = kPathY - 0.2, kBlockY1 = kPathY + 0.2;
constexpr double kBlockJudgedZ0 = kOff + 0.3, kBlockJudgedZ1 = kOff + 0.55;

struct Keyframe
{
  double x;
  bool crate;
  bool blocker = false;
};

bool onJudgedCrate(double x, double y, double z)
{
  constexpr double e = 0.01;
  return x > kCrateX0 - e && x < kCrateX1 + e && y > kCrateY0 - e && y < kCrateY1 + e &&
         z > kJudgedZ0 && z < kJudgedZ1;
}

bool onJudgedBlocker(double x, double y, double z)
{
  constexpr double e = 0.01;
  return x > kBlockX0 - e && x < kBlockX1 + e && std::fabs(y - kPathY) < 0.15 &&
         z > kBlockJudgedZ0 && z < kBlockJudgedZ1;
}

// Where a ray from o along d enters the box [lo, hi], or "never".
double boxEntry(const double o[3], const double d[3], const double lo[3], const double hi[3])
{
  double t_in = 0.0, t_out = std::numeric_limits<double>::max();
  for (int a = 0; a < 3; ++a) {
    if (std::fabs(d[a]) < 1e-9) {
      if (o[a] < lo[a] || o[a] > hi[a]) return std::numeric_limits<double>::max();
      continue;
    }
    const double t0 = (lo[a] - o[a]) / d[a], t1 = (hi[a] - o[a]) / d[a];
    t_in = std::max(t_in, std::min(t0, t1));
    t_out = std::min(t_out, std::max(t0, t1));
  }
  return t_in < t_out ? t_in : std::numeric_limits<double>::max();
}

// One scan from (x, kPathY, floor + lidar height), body frame = world axes:
// 0.6 degree rays against the room and, with `crate` / `blocker`, those.
pcl::PointCloud<pcl::PointXYZI> scan(const Keyframe & kf)
{
  const double ox = kf.x, oy = kPathY, oz = kOff + kLidarHeight;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  constexpr double kStep = 0.6 * M_PI / 180.0;
  for (double el = -45.0 * M_PI / 180.0; el <= 30.0 * M_PI / 180.0; el += kStep) {
    for (double az = 0.0; az < 2.0 * M_PI; az += kStep) {
      const double d[3] = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
      const double o[3] = {ox, oy, oz};
      // The room from inside: the nearest wall, floor or ceiling ahead.
      const double lo[3] = {kOff, kOff, kOff};
      const double hi[3] = {kOff + kRoomX, kOff + kRoomY, kOff + kRoomZ};
      double t = std::numeric_limits<double>::max();
      for (int a = 0; a < 3; ++a) {
        if (d[a] > 1e-9) t = std::min(t, (hi[a] - o[a]) / d[a]);
        if (d[a] < -1e-9) t = std::min(t, (lo[a] - o[a]) / d[a]);
      }
      if (kf.crate) {
        // The crate from outside: slab test.
        const double clo[3] = {kCrateX0, kCrateY0, kOff};
        const double chi[3] = {kCrateX1, kCrateY1, kCrateZ1};
        double t_in = 0.0, t_out = std::numeric_limits<double>::max();
        for (int a = 0; a < 3; ++a) {
          if (std::fabs(d[a]) < 1e-9) {
            if (o[a] < clo[a] || o[a] > chi[a]) t_in = std::numeric_limits<double>::max();
            continue;
          }
          const double t0 = (clo[a] - o[a]) / d[a], t1 = (chi[a] - o[a]) / d[a];
          t_in = std::max(t_in, std::min(t0, t1));
          t_out = std::min(t_out, std::max(t0, t1));
        }
        if (t_in < t_out && t_in < t) t = t_in;
      }
      if (kf.blocker) {
        const double blo[3] = {kBlockX0, kBlockY0, kOff};
        const double bhi[3] = {kBlockX1, kBlockY1, kOff + 1.0};
        t = std::min(t, boxEntry(o, d, blo, bhi));
      }
      pcl::PointXYZI p;
      p.x = static_cast<float>(d[0] * t);
      p.y = static_cast<float>(d[1] * t);
      p.z = static_cast<float>(d[2] * t);
      p.intensity = 1.0f;
      cloud.push_back(p);
    }
  }
  return cloud;
}

// `n` keyframes evenly along the path from x0 to x1; one stands halfway.
std::vector<Keyframe> walk(size_t n, double x0, double x1, bool crate, bool blocker = false)
{
  std::vector<Keyframe> ret;
  for (size_t i = 0; i < n; ++i) {
    const double along = n > 1 ? static_cast<double>(i) / static_cast<double>(n - 1) : 0.5;
    ret.push_back({kOff + x0 + (x1 - x0) * along, crate, blocker});
  }
  return ret;
}

// The length of the room, looking through where the crate stands from both
// sides of it.
std::vector<Keyframe> wholeRoom(size_t n, bool crate)
{
  return walk(n, 1.0, 7.0, crate);
}

// In front of the crate only, so that every one of them sees its near face.
std::vector<Keyframe> pastTheCrate(size_t n, bool crate)
{
  return walk(n, 3.6, 4.4, crate);
}

struct Outcome
{
  dr::Measurements m;
  size_t crate_points_saved = 0;  // on the judged part, in the patches
  size_t crate_points_kept = 0;   // of those, in the cleaned map.pcd
  size_t face_voxels = 0;         // sampled on the crate's near face
  size_t face_voxels_occupied = 0;
  size_t face_voxels_free = 0;
  size_t blocker_points_saved = 0;
  size_t blocker_points_kept = 0;
  size_t blocker_voxels = 0;  // sampled on the blocker's face toward the robot
  size_t blocker_voxels_occupied = 0;
  size_t blocker_voxels_free = 0;
  double keptShare() const
  {
    return static_cast<double>(crate_points_kept) / static_cast<double>(crate_points_saved);
  }
  double blockerKeptShare() const
  {
    return static_cast<double>(blocker_points_kept) / static_cast<double>(blocker_points_saved);
  }
};

class MapCleanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    m_dir = fs::temp_directory_path() /
            ("syncai_map_clean_" + std::to_string(::getpid()) + "_" + info->name());
    fs::remove_all(m_dir);
    fs::create_directories(m_dir / "patches");
  }
  void TearDown() override { fs::remove_all(m_dir); }

  // A save of these runs, as save_maps writes one: patches/, poses.txt and,
  // for more than one run, sessions.json.
  void save(const std::vector<std::vector<Keyframe>> & runs)
  {
    std::ofstream poses(m_dir / "poses.txt");
    std::vector<size_t> starts;
    size_t i = 0;
    m_crate_points_saved = 0;
    m_blocker_points_saved = 0;
    for (const auto & run : runs) {
      starts.push_back(i);
      for (const Keyframe & kf : run) {
        const pcl::PointCloud<pcl::PointXYZI> cloud = scan(kf);
        for (const auto & p : cloud) {
          if (onJudgedCrate(p.x + kf.x, p.y + kPathY, p.z + kOff + kLidarHeight)) {
            ++m_crate_points_saved;
          }
          if (onJudgedBlocker(p.x + kf.x, p.y + kPathY, p.z + kOff + kLidarHeight)) {
            ++m_blocker_points_saved;
          }
        }
        const std::string name = std::to_string(i++) + ".pcd";
        ASSERT_EQ(pcl::io::savePCDFileBinary((m_dir / "patches" / name).string(), cloud), 0);
        poses << name << " " << kf.x << " " << kPathY << " " << kOff + kLidarHeight << " 1 0 0 0\n";
      }
    }
    poses.close();
    writeSessions(m_dir, i, starts);
  }

  Outcome clean(double track_clear_radius = dr::Params{}.track_clear_radius)
  {
    Outcome out;
    dr::Params params;
    params.octomap = true;
    params.track_clear_radius = track_clear_radius;
    EXPECT_TRUE(dr::clean(m_dir, params, out.m));
    EXPECT_TRUE(out.m.octomap_written) << out.m.octomap_error;
    out.crate_points_saved = m_crate_points_saved;
    out.blocker_points_saved = m_blocker_points_saved;

    pcl::PointCloud<pcl::PointXYZI> map;
    EXPECT_EQ(pcl::io::loadPCDFile((m_dir / dr::kMapFile).string(), map), 0);
    for (const auto & p : map) {
      if (onJudgedCrate(p.x, p.y, p.z)) ++out.crate_points_kept;
      if (onJudgedBlocker(p.x, p.y, p.z)) ++out.blocker_points_kept;
    }

    octomap::OcTree tree((m_dir / dr::kOctomapFile).string());
    for (double x = kCrateX0 + 0.1; x < kCrateX1 - 0.05; x += 0.1) {
      for (double z = kJudgedZ0; z < kJudgedZ1; z += 0.1) {
        ++out.face_voxels;
        const auto * node = tree.search(x, kCrateY0 + 0.02, z);
        if (node) (tree.isNodeOccupied(node) ? out.face_voxels_occupied : out.face_voxels_free)++;
      }
    }
    for (double y = kPathY - 0.1; y < kPathY + 0.15; y += 0.1) {
      for (double z = kBlockJudgedZ0 + 0.05; z < kBlockJudgedZ1; z += 0.1) {
        ++out.blocker_voxels;
        const auto * node = tree.search(kBlockX0 + 0.02, y, z);
        if (node) {
          (tree.isNodeOccupied(node) ? out.blocker_voxels_occupied : out.blocker_voxels_free)++;
        }
      }
    }
    return out;
  }

  fs::path m_dir;
  size_t m_crate_points_saved = 0;
  size_t m_blocker_points_saved = 0;
};

// The robot walks the room's length, through where the blocker will stand;
// then, back at the start, it waits while someone stands there: 12 scans hit
// the blocker, the walk's 10 looked through it.
std::vector<Keyframe> walkThenWaitBehindTheBlocker()
{
  std::vector<Keyframe> kfs = wholeRoom(10, false);
  const std::vector<Keyframe> wait = walk(12, 1.5, 2.5, false, true);
  kfs.insert(kfs.end(), wait.begin(), wait.end());
  return kfs;
}

// The case this is for: the saved map never saw the crate, the resumed run
// finds it there.
TEST_F(MapCleanTest, WhatArrivedBetweenTwoRunsIsKept)
{
  save({wholeRoom(20, false), pastTheCrate(4, true)});
  const Outcome o = clean();
  EXPECT_EQ(o.m.sessions, 2u);
  EXPECT_TRUE(o.m.sessions_warning.empty()) << o.m.sessions_warning;
  ASSERT_GT(o.crate_points_saved, 1000u);
  EXPECT_GT(o.keptShare(), 0.9);
  EXPECT_GT(o.m.session_kept_voxels, 0u);
  // The planner must not walk through it.
  EXPECT_EQ(o.face_voxels_occupied, o.face_voxels);
}

// The same keyframes with nothing to tell the runs apart are one run in which
// 4 scans hit something 20 looked through: a person. This is the clean a
// resumed map got before it read sessions.json, and still what a map whose
// sessions.json is lost gets.
TEST_F(MapCleanTest, TheSameKeyframesAsOneRunLoseIt)
{
  save({wholeRoom(20, false), pastTheCrate(4, true)});
  fs::remove(m_dir / kSessionsFile);
  const Outcome o = clean();
  EXPECT_EQ(o.m.sessions, 1u);
  EXPECT_EQ(o.m.session_kept_voxels, 0u);
  EXPECT_LT(o.keptShare(), 0.1);
  EXPECT_EQ(o.face_voxels_free, o.face_voxels);
}

TEST_F(MapCleanTest, WhatLeftBetweenTwoRunsIsRemoved)
{
  save({pastTheCrate(4, true), wholeRoom(20, false)});
  const Outcome o = clean();
  EXPECT_EQ(o.m.sessions, 2u);
  EXPECT_EQ(o.m.session_kept_voxels, 0u);
  EXPECT_LT(o.keptShare(), 0.1);
  EXPECT_EQ(o.face_voxels_free, o.face_voxels);
}

// Two scans are what the sparse rule asks of anything, so they are enough to
// stand on. They are not enough to lift the tree's log-odds back over the
// threshold from where 20 scans looking through left it, which is why such a
// voxel is set occupied rather than left to the sum.
TEST_F(MapCleanTest, TwoScansOfALaterRunAreEnough)
{
  save({wholeRoom(20, false), pastTheCrate(2, true)});
  const Outcome o = clean();
  EXPECT_GT(o.keptShare(), 0.9);
  EXPECT_EQ(o.face_voxels_occupied, o.face_voxels);
}

// Someone standing there for two scans of the resumed run, which then walks on
// and looks through the spot: the later run does not stand for them.
TEST_F(MapCleanTest, APersonInTheLaterRunIsRemoved)
{
  std::vector<Keyframe> second = pastTheCrate(2, true);
  const std::vector<Keyframe> rest = wholeRoom(12, false);
  second.insert(second.end(), rest.begin(), rest.end());
  save({wholeRoom(10, false), second});
  const Outcome o = clean();
  EXPECT_EQ(o.m.sessions, 2u);
  EXPECT_EQ(o.m.session_kept_voxels, 0u);
  EXPECT_LT(o.keptShare(), 0.1);
  EXPECT_EQ(o.face_voxels_free, o.face_voxels);
}

// And in the saved map's run, where the resumed one finds the spot empty.
TEST_F(MapCleanTest, APersonInTheEarlierRunIsRemoved)
{
  std::vector<Keyframe> first = pastTheCrate(2, true);
  const std::vector<Keyframe> rest = wholeRoom(12, false);
  first.insert(first.end(), rest.begin(), rest.end());
  save({first, wholeRoom(6, false)});
  const Outcome o = clean();
  EXPECT_EQ(o.m.session_kept_voxels, 0u);
  EXPECT_LT(o.keptShare(), 0.1);
}

// One scan of a later run is a one-off return, not a thing that arrived: it
// does not bring back what every earlier scan looked through.
TEST_F(MapCleanTest, OneScanOfALaterRunStandsForNothing)
{
  save({wholeRoom(20, false), pastTheCrate(1, true)});
  const Outcome o = clean();
  EXPECT_EQ(o.m.sessions, 2u);
  EXPECT_EQ(o.m.session_kept_voxels, 0u);
  EXPECT_LT(o.keptShare(), 0.1);
}

// Something that was always there is not the dynamic rule's at all, whatever
// the runs.
TEST_F(MapCleanTest, WhatStoodThroughBothRunsIsKept)
{
  save({wholeRoom(12, true), wholeRoom(6, true)});
  const Outcome o = clean();
  EXPECT_EQ(o.m.session_kept_voxels, 0u);
  EXPECT_GT(o.keptShare(), 0.9);
  EXPECT_EQ(o.face_voxels_occupied, o.face_voxels);
}

// A sessions.json that does not describe this poses.txt is not half-used.
TEST_F(MapCleanTest, ASessionsFileThatDoesNotFitIsIgnored)
{
  save({wholeRoom(20, false), pastTheCrate(4, true)});
  std::ofstream(m_dir / kSessionsFile, std::ios::trunc)
    << R"({"version": 1, "sessions": [{"first": 0, "count": 20}, {"first": 20, "count": 9}]})";
  const Outcome o = clean();
  EXPECT_EQ(o.m.sessions, 1u);
  EXPECT_NE(o.m.sessions_warning.find("ignored"), std::string::npos) << o.m.sessions_warning;
  EXPECT_LT(o.keptShare(), 0.1);
}

// The case the path rule is for: on the robot's path, standing longer than
// the dynamic rule's ratio allows.
TEST_F(MapCleanTest, SomeoneWhoStoodOnThePathIsRemoved)
{
  save({walkThenWaitBehindTheBlocker()});
  const Outcome o = clean();
  ASSERT_GT(o.blocker_points_saved, 100u);
  EXPECT_GT(o.m.track_cleared_voxels, 0u);
  EXPECT_LT(o.blockerKeptShare(), 0.1);
  // Free, not unknown: the planner must be able to walk there again.
  EXPECT_EQ(o.blocker_voxels_free, o.blocker_voxels);
}

// The same scans without the path rule: the dynamic rule alone keeps it, which
// is what dp_1125's stair foot got.
TEST_F(MapCleanTest, WithoutThePathRuleItStays)
{
  save({walkThenWaitBehindTheBlocker()});
  const Outcome o = clean(0.0);
  EXPECT_EQ(o.m.track_cleared_voxels, 0u);
  EXPECT_GT(o.blockerKeptShare(), 0.9);
  EXPECT_EQ(o.blocker_voxels_occupied, o.blocker_voxels);
}

// The robot's path proves nothing about a place it never reached: the walk
// stops short of the blocker, which stays.
TEST_F(MapCleanTest, WhatThePathNeverReachedStays)
{
  save({walk(10, 1.0, 4.0, false, true)});
  const Outcome o = clean();
  EXPECT_GT(o.blockerKeptShare(), 0.9);
  EXPECT_EQ(o.blocker_voxels_occupied, o.blocker_voxels);
}

// Off the path (the crate stands 2 m from it, always there): untouched.
TEST_F(MapCleanTest, ThePathRuleLeavesWhatIsBesideThePath)
{
  save({wholeRoom(12, true)});
  const Outcome o = clean();
  EXPECT_EQ(o.m.track_cleared_voxels, 0u);
  EXPECT_GT(o.keptShare(), 0.9);
}

}  // namespace

// A resumed run on the synthetic building (synthetic_hall.h). The saved map is one lap of the
// rectangle, at its ground-truth poses. The new run starts at the opposite corner, in an odometry
// frame of its own (origin at the robot, yaw 0, as after a LIO reset), with drifting odometry and
// a map <- odom offset that is a few centimetres and a fraction of a degree off, as a session
// alignment leaves it. The saved keyframes must not move by one bit, and the new ones must end up
// in their frame.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "syncai_mapping/pgos/pose_graph_optimizer.h"
#include "synthetic_hall.h"

namespace
{
using namespace synthetic_hall;

const World & world()
{
  static const World w;
  return w;
}

// The saved map, as readSavedMap() returns one: global poses and body clouds, nothing else.
const std::vector<KeyPoseWithCloud> & savedMap()
{
  static const std::vector<KeyPoseWithCloud> map = [] {
    std::mt19937 rng(21);
    std::vector<KeyPoseWithCloud> ret;
    for (const Pose2 & p : groundTruthPath(1.0)) {
      KeyPoseWithCloud kp;
      kp.r_global = Eigen::AngleAxisd(p.yaw, V3D::UnitZ()).toRotationMatrix();
      kp.t_global = V3D(p.x, p.y, kSensorHeight);
      kp.body_cloud = world().scan(p, rng);
      ret.push_back(kp);
    }
    return ret;
  }();
  return map;
}

// The new run's ground truth: from the corner opposite the saved map's start, along two sides and
// into the third.
std::vector<Pose2> newRunPath()
{
  return rectanglePath(Pose2{15.0, 8.0, M_PI}, 60.0);
}

struct ResumeResult
{
  size_t prior = 0;
  size_t key_poses = 0;
  int prior_loops = 0;             // loops against a saved keyframe
  int prior_sc_candidates = 0;     // Scan Context candidates against one that passed every gate
  int first_prior_loop = -1;       // the run's keyframe (0 = its first) that closed the first one
  bool prior_untouched = false;    // every saved pose bit-identical afterwards
  double open_loop_end_error = 0;  // m, the odometry through the loaded offset, last keyframe
  double graph_end_error = 0;      // m, xy
  double graph_max_error = 0;      // m, xy, over the run's keyframes
  double graph_max_z_error = 0;    // m, over the run's keyframes
  std::vector<size_t> session_starts;
};

// odom_pitch tilts the run's whole odometry frame about its y axis: the LIO's up a little off the
// saved map's, as two static IMU initialisations leave two runs. With offset_has_tilt the loaded
// map <- odom carries that tilt, as the session alignment returns it; without, it is the rotation
// about z of an alignment that missed it, which leaves the gravity anchors off by that much and
// the track climbing as it goes.
ResumeResult resume(
  PoseGraphOptimizerConfig config, double yaw_drift_per_step, double odom_pitch,
  bool offset_has_tilt = true)
{
  const std::vector<Pose2> gt = newRunPath();
  const std::vector<KeyPoseWithCloud> & saved = savedMap();
  ResumeResult result;
  result.prior = saved.size();

  // The true map <- odom: out of the tilted odom, then the run's first pose. What is loaded is a
  // little off it.
  const M3D tilt = Eigen::AngleAxisd(odom_pitch, V3D::UnitY()).toRotationMatrix();
  const V3D t_true(gt.front().x, gt.front().y, kSensorHeight);
  M3D r_offset =
    Eigen::AngleAxisd(gt.front().yaw + 0.3 * M_PI / 180.0, V3D::UnitZ()).toRotationMatrix();
  if (offset_has_tilt) r_offset = r_offset * tilt.transpose();
  const V3D t_offset = t_true + V3D(0.04, -0.03, 0.0);

  PoseGraphOptimizer pgo(config);
  pgo.loadPrior(std::vector<KeyPoseWithCloud>(saved), {0}, r_offset, t_offset);
  EXPECT_EQ(pgo.priorCount(), saved.size());

  // The ground truth seen from the run's first pose: what its odometry would read without drift.
  std::vector<Pose2> local(gt.size());
  for (size_t i = 0; i < gt.size(); i++) {
    const double c = std::cos(gt.front().yaw), s = std::sin(gt.front().yaw);
    const double dx = gt[i].x - gt.front().x, dy = gt[i].y - gt.front().y;
    local[i] = Pose2{c * dx + s * dy, -s * dx + c * dy, gt[i].yaw - gt.front().yaw};
  }

  std::mt19937 rng(31);
  Pose2 odom{0.0, 0.0, 0.0};
  V3D last_local_t = V3D::Zero();
  for (size_t i = 0; i < gt.size(); i++) {
    if (i > 0) stepOdometry(odom, local[i - 1], local[i], yaw_drift_per_step);
    CloudWithPose cp;
    cp.cloud = world().scan(gt[i], rng);
    cp.pose.t = tilt * V3D(odom.x, odom.y, 0.0);
    cp.pose.r = tilt * Eigen::AngleAxisd(odom.yaw, V3D::UnitZ()).toRotationMatrix();
    cp.pose.second = static_cast<double>(i);  // 1 s per keyframe, from 0: a new clock
    last_local_t = cp.pose.t;
    if (!pgo.addKeyPose(cp)) continue;
    pgo.searchForLoopPairs();
    for (const LoopCandidateReport & c : pgo.lastCandidates())
      if (c.accepted && c.scan_context && pgo.isPrior(c.target_id)) result.prior_sc_candidates++;
    for (const LoopPair & lp : pgo.pendingLoops()) {
      if (!pgo.isPrior(lp.target_id)) continue;
      result.prior_loops++;
      if (result.first_prior_loop < 0)
        result.first_prior_loop = static_cast<int>(lp.source_id - saved.size());
    }
    pgo.smoothAndUpdate();
  }

  result.key_poses = pgo.keyPoses().size();
  result.session_starts = pgo.sessionStarts();
  EXPECT_EQ(result.key_poses, saved.size() + gt.size()) << "every step should be a keyframe";

  result.prior_untouched = true;
  for (size_t i = 0; i < saved.size(); i++) {
    const KeyPoseWithCloud & kp = pgo.keyPoses()[i];
    if (
      std::memcmp(kp.r_global.data(), saved[i].r_global.data(), 9 * sizeof(double)) != 0 ||
      std::memcmp(kp.t_global.data(), saved[i].t_global.data(), 3 * sizeof(double)) != 0)
      result.prior_untouched = false;
  }
  for (size_t i = 0; i < gt.size(); i++) {
    const V3D & t = pgo.keyPoses()[saved.size() + i].t_global;
    result.graph_max_error =
      std::max(result.graph_max_error, std::hypot(t.x() - gt[i].x, t.y() - gt[i].y));
    result.graph_max_z_error = std::max(result.graph_max_z_error, std::abs(t.z() - kSensorHeight));
  }
  const V3D & last = pgo.keyPoses().back().t_global;
  result.graph_end_error = std::hypot(last.x() - gt.back().x, last.y() - gt.back().y);
  const V3D open_loop = r_offset * last_local_t + t_offset;
  result.open_loop_end_error = std::hypot(open_loop.x() - gt.back().x, open_loop.y() - gt.back().y);
  return result;
}

void print(const char * name, const ResumeResult & r)
{
  std::printf(
    "[%s] %zu saved + %zu new keyframes | loops against the saved map %d (scan context %d, first "
    "at new keyframe %d) | end error open loop %.3f m graph %.3f m | max graph %.3f m, z %.3f m "
    "| saved poses %s\n",
    name, r.prior, r.key_poses - r.prior, r.prior_loops, r.prior_sc_candidates, r.first_prior_loop,
    r.open_loop_end_error, r.graph_end_error, r.graph_max_error, r.graph_max_z_error,
    r.prior_untouched ? "untouched" : "MOVED");
}

void expectResumed(const ResumeResult & r)
{
  EXPECT_TRUE(r.prior_untouched);
  EXPECT_EQ(r.session_starts, (std::vector<size_t>{0, r.prior}));
  EXPECT_GT(r.prior_loops, 0);
  // A saved keyframe has no stamp to be loop_time_tresh (60 s) older than: the first loop comes
  // as soon as the search runs, not a minute into the run.
  EXPECT_GE(r.first_prior_loop, 0);
  EXPECT_LT(r.first_prior_loop, 15);
  EXPECT_LT(r.graph_max_error, 0.3);
  EXPECT_LT(r.graph_end_error, 0.5 * r.open_loop_end_error);
}

PoseGraphOptimizerConfig baseConfig()
{
  PoseGraphOptimizerConfig config;
  config.keyframe_delta_m = 0.5;
  return config;
}
}  // namespace

TEST(ResumeMapping, NewRunIsTiedToTheSavedMap)
{
  const ResumeResult r = resume(baseConfig(), 0.0006, 0.0);
  print("resume", r);
  expectResumed(r);
}

// With the tilt anchors off, only the run's first keyframe has anything holding its roll and
// pitch (in a run begun empty, node 0's 6-DOF prior does it). Must solve, not throw.
TEST(ResumeMapping, SolvesWithoutTiltAnchors)
{
  PoseGraphOptimizerConfig config = baseConfig();
  config.keyframe_tilt_sigma_deg = 0.0;
  const ResumeResult r = resume(config, 0.0006, 0.0);
  print("no tilt anchors", r);
  expectResumed(r);
}

// The 6-DOF loop edge against a constant target: a prior at target * measurement.
TEST(ResumeMapping, SixDofLoopsAgainstTheSavedMap)
{
  PoseGraphOptimizerConfig config = baseConfig();
  config.loop_planar_correction = false;
  const ResumeResult r = resume(config, 0.0006, 0.0);
  print("6-DOF loops", r);
  expectResumed(r);
}

// Scan Context needs a descriptor on the saved keyframes too, built when they are loaded. The
// radius search is switched off as in test_loop_closure (a 1 cm radius, not drift-scaled), so
// every loop here is one Scan Context found against the saved map.
TEST(ResumeMapping, ScanContextSeesTheSavedMap)
{
  PoseGraphOptimizerConfig config = baseConfig();
  config.loop_search_radius = 0.01;
  config.loop_radius_drift_scaled = false;
  const ResumeResult without = resume(config, 0.0006, 0.0);
  print("radius off", without);
  EXPECT_EQ(without.prior_loops, 0);

  config.loop_sc_enabled = true;
  const ResumeResult with = resume(config, 0.0006, 0.0);
  print("radius off, scan context on", with);
  EXPECT_TRUE(with.prior_untouched);
  EXPECT_GT(with.prior_sc_candidates, 0);
  EXPECT_EQ(with.prior_loops, with.prior_sc_candidates);
  EXPECT_LT(with.graph_end_error, 0.5 * with.open_loop_end_error);
  EXPECT_LT(with.graph_max_error, without.graph_max_error);
}

// A run whose LIO levelled itself 0.8 deg off the saved map (robot01 measured 0.76 between two of
// its runs). The alignment's transform carries the tilt, each keyframe is anchored to the saved
// map's up, and the run stays on the saved map's floor; anchored to its own up it climbed
// 1.4 cm a metre.
TEST(ResumeMapping, ATiltedRunTakesTheSavedMapsUp)
{
  const double pitch = -0.8 * M_PI / 180.0;  // about +y, negative: the track climbs along +x
  const ResumeResult r = resume(baseConfig(), 0.0006, pitch);
  print("tilted run, the saved map's up", r);
  expectResumed(r);
  EXPECT_LT(r.graph_max_z_error, 0.03);

  const ResumeResult own_up = resume(baseConfig(), 0.0006, pitch, false);
  print("tilted run, its own up", own_up);
  EXPECT_GT(own_up.graph_max_z_error, 0.3);
}

// What an alignment leaves of the tilt, here 0.4 deg of it, still climbs 0.7 cm a metre. Planar
// loops leave that in; with resume_loop_z_sigma_m each loop against the saved map also holds its
// keyframe's height.
TEST(ResumeMapping, LoopHeightHoldsATiltedRun)
{
  const double pitch = -0.4 * M_PI / 180.0;  // about +y, negative: the track climbs along +x
  PoseGraphOptimizerConfig config = baseConfig();
  const ResumeResult planar = resume(config, 0.0006, pitch, false);
  print("tilted run, planar loops", planar);
  EXPECT_TRUE(planar.prior_untouched);
  EXPECT_GT(planar.graph_max_z_error, 0.15);

  config.resume_loop_z_sigma_m = 0.05;
  const ResumeResult held = resume(config, 0.0006, pitch, false);
  print("tilted run, loop height on", held);
  EXPECT_TRUE(held.prior_untouched);
  EXPECT_GT(held.prior_loops, 0);
  EXPECT_LT(held.graph_max_z_error, 0.5 * planar.graph_max_z_error);
  EXPECT_LT(held.graph_max_error, 0.3);
}

// The two unary factors at a pose pitched like the lidar: errors in world terms, whatever the
// body's attitude.
TEST(ResumeMapping, AnchorFactorsAreWorldFrame)
{
  const double yaw = 40.0 * M_PI / 180.0;
  const gtsam::Rot3 rot(
    (Eigen::AngleAxisd(yaw, V3D::UnitZ()) * Eigen::AngleAxisd(15.0 * M_PI / 180.0, V3D::UnitY()))
      .toRotationMatrix());
  const gtsam::Pose3 pose(rot, gtsam::Point3(3.0, 4.0, 5.0));
  const PlanarAnchorFactor planar(
    0, gtsam::Vector3(3.0, 4.0, yaw), gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  const HeightPriorFactor height(0, 5.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
  EXPECT_LT(planar.evaluateError(pose).norm(), 1e-9);
  EXPECT_LT(height.evaluateError(pose).norm(), 1e-9);

  // Moved in world x and z, and turned about world z.
  const double dyaw = 0.05;
  const gtsam::Pose3 moved(
    gtsam::Rot3(Eigen::AngleAxisd(dyaw, V3D::UnitZ()).toRotationMatrix()) * rot,
    gtsam::Point3(3.1, 4.0, 5.3));
  const gtsam::Vector e = planar.evaluateError(moved);
  EXPECT_NEAR(e(0), 0.1, 1e-9);
  EXPECT_NEAR(e(1), 0.0, 1e-9);
  EXPECT_NEAR(e(2), dyaw, 1e-9);
  EXPECT_NEAR(height.evaluateError(moved)(0), 0.3, 1e-9);

  // The Jacobians: a body-frame x step moves world x, y AND z here, and each factor sees only
  // its own part of it.
  gtsam::Matrix h_planar, h_height;
  planar.evaluateError(pose, h_planar);
  height.evaluateError(pose, h_height);
  ASSERT_EQ(h_planar.rows(), 3);
  ASSERT_EQ(h_planar.cols(), 6);
  ASSERT_EQ(h_height.rows(), 1);
  const V3D body_x = rot.matrix().col(0);  // Pose3's tangent: (rotation, translation), body frame
  EXPECT_NEAR(h_planar(0, 3), body_x.x(), 1e-6);
  EXPECT_NEAR(h_planar(1, 3), body_x.y(), 1e-6);
  EXPECT_NEAR(h_planar(2, 3), 0.0, 1e-6);
  EXPECT_NEAR(h_height(0, 3), body_x.z(), 1e-6);
  EXPECT_GT(std::abs(body_x.z()), 0.2);
}

// A run begun empty is what it was: loadPrior with nothing is no resume.
TEST(ResumeMapping, NothingLoadedIsAnEmptyRun)
{
  PoseGraphOptimizer pgo(baseConfig());
  pgo.loadPrior({}, {0}, M3D::Identity(), V3D::Zero());
  EXPECT_EQ(pgo.priorCount(), 0u);
  EXPECT_EQ(pgo.sessionStarts(), std::vector<size_t>{0});
}

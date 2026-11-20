// Loop closure end to end on a synthetic building (synthetic_hall.h): the
// hall driven around its 30 x 16 m rectangle 1.3 times with odometry whose yaw
// drifts by a constant rate. The graph sees only the drifted odometry and the
// keyframes' clouds.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "syncai_mapping/pgos/pose_graph_optimizer.h"
#include "synthetic_hall.h"

namespace
{
using namespace synthetic_hall;

struct RunResult
{
  int loops = 0;
  int scan_context_loops = 0;
  int coarse_radius_loops = 0;   // accepted radius candidates beyond loop_search_radius
  double odom_end_error = 0.0;   // m, the drifted odometry at the last keyframe
  double graph_end_error = 0.0;  // m, the optimised pose at the last keyframe
  double graph_max_error = 0.0;  // m, over all keyframes
};

RunResult drive(const World & world, PoseGraphOptimizerConfig config, double yaw_drift_per_step)
{
  const std::vector<Pose2> gt = groundTruthPath(1.3);
  std::mt19937 rng(11);
  PoseGraphOptimizer pgo(config);
  RunResult result;

  Pose2 odom = gt.front();
  for (size_t i = 0; i < gt.size(); i++) {
    if (i > 0) stepOdometry(odom, gt[i - 1], gt[i], yaw_drift_per_step);
    CloudWithPose cp;
    cp.cloud = world.scan(gt[i], rng);
    cp.pose.t = V3D(odom.x, odom.y, kSensorHeight);
    cp.pose.r = Eigen::AngleAxisd(odom.yaw, V3D::UnitZ()).toRotationMatrix();
    cp.pose.second = static_cast<double>(i);  // 1 s per keyframe
    if (!pgo.addKeyPose(cp)) continue;
    pgo.searchForLoopPairs();
    result.loops += static_cast<int>(pgo.pendingLoops().size());
    for (const LoopCandidateReport & c : pgo.lastCandidates()) {
      if (c.accepted && c.scan_context) result.scan_context_loops++;
      if (c.accepted && !c.scan_context && c.coarse) result.coarse_radius_loops++;
      if (std::getenv("LOOP_DEBUG"))
        std::printf(
          "  cand %zu->%zu sc=%d coarse=%d acc=%d gt_sep %.2f m gt_dyaw %.1f deg | sc %.3f ov %.2f "
          "in %.2f "
          "fit %.4f xy %.3f corr %.2f/%.2f %s\n",
          c.target_id, c.source_id, c.scan_context, c.coarse, c.accepted,
          std::hypot(gt[c.target_id].x - gt[c.source_id].x, gt[c.target_id].y - gt[c.source_id].y),
          (gt[c.source_id].yaw - gt[c.target_id].yaw) * 180.0 / M_PI, c.sc_dist, c.overlap_ratio,
          c.inlier_ratio, c.fitness, c.xy_info_ratio, c.correction_m, c.allowed_m, c.reject_reason);
    }
    pgo.smoothAndUpdate();
  }

  EXPECT_EQ(pgo.keyPoses().size(), gt.size()) << "every step should be a keyframe";
  for (size_t i = 0; i < gt.size(); i++) {
    const V3D & t = pgo.keyPoses()[i].t_global;
    const double e = std::hypot(t.x() - gt[i].x, t.y() - gt[i].y);
    result.graph_max_error = std::max(result.graph_max_error, e);
  }
  const V3D & last = pgo.keyPoses().back().t_global;
  result.graph_end_error = std::hypot(last.x() - gt.back().x, last.y() - gt.back().y);
  result.odom_end_error = std::hypot(odom.x - gt.back().x, odom.y - gt.back().y);
  return result;
}

const World & world()
{
  static const World w;
  return w;
}

void print(const char * name, const RunResult & r)
{
  std::printf(
    "[%s] loops %d (scan context %d, coarse radius %d) | end error odom %.3f m graph %.3f m | max "
    "graph %.3f m\n",
    name, r.loops, r.scan_context_loops, r.coarse_radius_loops, r.odom_end_error, r.graph_end_error,
    r.graph_max_error);
}
}  // namespace

// Half a metre of drift at the revisit: the radius search's case.
TEST(LoopClosure, RadiusSearchClosesSmallDrift)
{
  PoseGraphOptimizerConfig config;
  config.keyframe_delta_m = 0.5;
  const RunResult r = drive(world(), config, 0.00015);
  print("radius", r);
  EXPECT_GT(r.loops, 0);
  EXPECT_EQ(r.scan_context_loops, 0);
  EXPECT_LT(r.graph_end_error, 0.5 * r.odom_end_error);
}

// ~2 m of drift at the revisit. On this path the radius search still closes
// it where the second pass crosses the first, so it is switched off here
// (a 1 cm radius, not drift-scaled, never finds anything) to test Scan
// Context alone.
TEST(LoopClosure, ScanContextClosesLargeDrift)
{
  PoseGraphOptimizerConfig config;
  config.keyframe_delta_m = 0.5;
  config.loop_search_radius = 0.01;
  config.loop_radius_drift_scaled = false;
  const RunResult without = drive(world(), config, 0.0006);
  print("large drift, radius only", without);
  EXPECT_EQ(without.loops, 0);

  config.loop_sc_enabled = true;
  const RunResult with = drive(world(), config, 0.0006);
  print("large drift, scan context", with);
  EXPECT_GT(with.scan_context_loops, 0);
  EXPECT_LT(with.graph_end_error, 0.5 * with.odom_end_error);
  EXPECT_LT(with.graph_max_error, with.odom_end_error);
}

// The same ~2 m of drift with Scan Context off and a 1 cm loop_search_radius:
// only the drift-scaled reach can find the revisit, and the candidate is
// metres from the estimate, so it must go through the coarse stage.
TEST(LoopClosure, DriftScaledRadiusClosesLargeDrift)
{
  PoseGraphOptimizerConfig config;
  config.keyframe_delta_m = 0.5;
  config.loop_search_radius = 0.01;
  config.loop_radius_drift_scaled = false;
  const RunResult fixed = drive(world(), config, 0.0006);
  print("large drift, fixed radius", fixed);
  EXPECT_EQ(fixed.loops, 0);

  config.loop_radius_drift_scaled = true;
  const RunResult scaled = drive(world(), config, 0.0006);
  print("large drift, drift-scaled radius", scaled);
  EXPECT_GT(scaled.coarse_radius_loops, 0);
  EXPECT_LT(scaled.graph_end_error, 0.5 * scaled.odom_end_error);
  EXPECT_LT(scaled.graph_max_error, scaled.odom_end_error);
}

// The hall is 180-deg symmetric: Scan Context finds the far side as well, and
// GICP aligns it with an inlier ratio of 0.6-0.8. Only the drift gate stops
// those loops; without it the map is torn apart.
TEST(LoopClosure, DriftGateRejectsSymmetricPlaces)
{
  PoseGraphOptimizerConfig config;
  config.keyframe_delta_m = 0.5;
  config.loop_sc_enabled = true;
  const RunResult gated = drive(world(), config, 0.0006);
  print("drift gate on", gated);
  EXPECT_LT(gated.graph_max_error, 1.0);

  config.loop_max_drift_ratio = 0.0;
  config.loop_robust_kernel = "none";
  const RunResult ungated = drive(world(), config, 0.0006);
  print("drift gate off, no kernel", ungated);
  EXPECT_GT(ungated.graph_max_error, 2.0 * gated.graph_max_error);
}

// The robust kernel must not cost a correct loop its correction: Huber is
// convex and smoothAndUpdate iterates until the estimate settles, so the
// result should match plain least squares.
TEST(LoopClosure, HuberMatchesLeastSquaresOnCorrectLoops)
{
  PoseGraphOptimizerConfig config;
  config.keyframe_delta_m = 0.5;
  config.loop_sc_enabled = true;
  config.loop_robust_kernel = "none";
  const RunResult plain = drive(world(), config, 0.0006);
  print("kernel none", plain);
  config.loop_robust_kernel = "huber";
  const RunResult huber = drive(world(), config, 0.0006);
  print("kernel huber", huber);
  EXPECT_GT(huber.loops, 0);
  EXPECT_LT(huber.graph_end_error, plain.graph_end_error + 0.05);
  EXPECT_LT(huber.graph_max_error, plain.graph_max_error + 0.05);
}

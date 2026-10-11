// Loop closure end to end on a synthetic building: a 40 x 24 m hall with
// pillars and furniture, driven around a 30 x 16 m rectangle 1.3 times with
// odometry whose yaw drifts by a constant rate. Every keyframe gets the
// points within the lidar range (no occlusion), in the body frame, with
// 1 cm noise. The graph sees only the drifted odometry and those clouds.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "syncai_mapping/pgos/pose_graph_optimizer.h"

namespace
{
constexpr double kSensorHeight = 0.5;
constexpr double kRange = 15.0;
constexpr double kStep = 0.55;  // > keyframe_delta_m, so every step is a keyframe

struct Pose2
{
  double x, y, yaw;
};

class World
{
public:
  World()
  {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> ux(-19.0, 19.0), uy(-11.0, 11.0), us(0.4, 1.2),
      uh(0.8, 3.0);
    for (double x = -20.0; x <= 20.0; x += 0.3)
      for (double y = -12.0; y <= 12.0; y += 0.3) add(x, y, 0.0);
    for (double z = 0.0; z <= 3.0; z += 0.15) {
      for (double x = -20.0; x <= 20.0; x += 0.15) {
        add(x, -12.0, z);
        add(x, 12.0, z);
      }
      for (double y = -12.0; y <= 12.0; y += 0.15) {
        add(-20.0, y, z);
        add(20.0, y, z);
      }
    }
    // Boxes of random footprint and height, kept off the driven rectangle.
    int boxes = 0;
    while (boxes < 40) {
      const double cx = ux(rng), cy = uy(rng), s = us(rng), h = uh(rng);
      const bool on_path_y = std::abs(std::abs(cy) - 8.0) < 1.5 + s && std::abs(cx) < 16.5 + s;
      const bool on_path_x = std::abs(std::abs(cx) - 15.0) < 1.5 + s && std::abs(cy) < 9.5 + s;
      if (on_path_x || on_path_y) continue;
      for (double z = 0.0; z <= h; z += 0.15)
        for (double a = -s; a <= s; a += 0.15) {
          add(cx + a, cy - s, z);
          add(cx + a, cy + s, z);
          add(cx - s, cy + a, z);
          add(cx + s, cy + a, z);
        }
      boxes++;
    }
  }

  // The scan from a ground-truth pose, in the body frame.
  CloudType::Ptr scan(const Pose2 & p, std::mt19937 & rng) const
  {
    std::normal_distribution<float> noise(0.0f, 0.01f);
    CloudType::Ptr cloud(new CloudType);
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    for (const V3D & w : m_points) {
      const double dx = w.x() - p.x, dy = w.y() - p.y;
      if (dx * dx + dy * dy > kRange * kRange) continue;
      PointType q;
      q.x = static_cast<float>(c * dx + s * dy) + noise(rng);
      q.y = static_cast<float>(-s * dx + c * dy) + noise(rng);
      q.z = static_cast<float>(w.z() - kSensorHeight) + noise(rng);
      q.intensity = 0.0f;
      cloud->push_back(q);
    }
    return cloud;
  }

private:
  void add(double x, double y, double z) { m_points.emplace_back(x, y, z); }
  std::vector<V3D> m_points;
};

// The ground-truth path: forward steps along the rectangle, each corner a
// left turn in place in 15 deg steps.
std::vector<Pose2> groundTruthPath(double laps)
{
  std::vector<Pose2> path;
  Pose2 p{-15.0, -8.0, 0.0};
  path.push_back(p);
  double travelled = 0.0;
  const double sides[4] = {30.0, 16.0, 30.0, 16.0};
  while (true) {
    for (double side : sides) {
      const int n = static_cast<int>(std::round(side / kStep));
      for (int i = 0; i < n; i++) {
        p.x += side / n * std::cos(p.yaw);
        p.y += side / n * std::sin(p.yaw);
        path.push_back(p);
        travelled += side / n;
        if (travelled >= 92.0 * laps) return path;
      }
      for (int i = 0; i < 6; i++) {
        p.yaw += 15.0 * M_PI / 180.0;
        path.push_back(p);
      }
    }
  }
}

struct RunResult
{
  int loops = 0;
  int scan_context_loops = 0;
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
    if (i > 0) {
      // Integrate the true body-frame increment with a yaw bias.
      const double c = std::cos(gt[i - 1].yaw), s = std::sin(gt[i - 1].yaw);
      const double dx = gt[i].x - gt[i - 1].x, dy = gt[i].y - gt[i - 1].y;
      const double fwd = c * dx + s * dy, left = -s * dx + c * dy;
      odom.yaw += gt[i].yaw - gt[i - 1].yaw + yaw_drift_per_step;
      odom.x += fwd * std::cos(odom.yaw) - left * std::sin(odom.yaw);
      odom.y += fwd * std::sin(odom.yaw) + left * std::cos(odom.yaw);
    }
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
      if (std::getenv("LOOP_DEBUG"))
        std::printf(
          "  cand %zu->%zu sc=%d acc=%d gt_sep %.2f m gt_dyaw %.1f deg | sc %.3f ov %.2f in %.2f "
          "fit %.4f xy %.3f corr %.2f/%.2f %s\n",
          c.target_id, c.source_id, c.scan_context, c.accepted,
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
    "[%s] loops %d (scan context %d) | end error odom %.3f m graph %.3f m | max graph %.3f m\n",
    name, r.loops, r.scan_context_loops, r.odom_end_error, r.graph_end_error, r.graph_max_error);
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
// (a 1 cm radius never finds anything) to test Scan Context alone.
TEST(LoopClosure, ScanContextClosesLargeDrift)
{
  PoseGraphOptimizerConfig config;
  config.keyframe_delta_m = 0.5;
  config.loop_search_radius = 0.01;
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

// The synthetic building the pose-graph tests drive through: a 40 x 24 m hall
// with furniture, and a robot walking a 30 x 16 m rectangle in it. Every
// keyframe gets the points within the lidar range (no occlusion), in the body
// frame, with 1 cm noise.
#pragma once

#include <cmath>
#include <random>
#include <vector>

#include "syncai_mapping/pgos/commons.h"

namespace synthetic_hall
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
  // Without the boxes the hall is exactly 180-deg symmetric about its centre:
  // a place and its mirror image see the same scan.
  explicit World(bool with_boxes = true)
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
    if (!with_boxes) return;
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

// A ground-truth path along the 30 x 16 m rectangle from one of its corners,
// `start` heading down its first (30 m) side: forward steps, each corner a
// left turn in place in 15 deg steps, until `length` metres are driven.
inline std::vector<Pose2> rectanglePath(Pose2 start, double length)
{
  std::vector<Pose2> path;
  Pose2 p = start;
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
        if (travelled >= length) return path;
      }
      for (int i = 0; i < 6; i++) {
        p.yaw += 15.0 * M_PI / 180.0;
        path.push_back(p);
      }
    }
  }
}

// From the (-15, -8) corner, `laps` times round (a lap is 92 m).
inline std::vector<Pose2> groundTruthPath(double laps)
{
  return rectanglePath(Pose2{-15.0, -8.0, 0.0}, 92.0 * laps);
}

// One step of the drifting odometry: the true body-frame increment from
// `prev` to `cur`, integrated onto `odom` with a yaw bias.
inline void stepOdometry(Pose2 & odom, const Pose2 & prev, const Pose2 & cur, double yaw_drift)
{
  const double c = std::cos(prev.yaw), s = std::sin(prev.yaw);
  const double dx = cur.x - prev.x, dy = cur.y - prev.y;
  const double fwd = c * dx + s * dy, left = -s * dx + c * dy;
  odom.yaw += cur.yaw - prev.yaw + yaw_drift;
  odom.x += fwd * std::cos(odom.yaw) - left * std::sin(odom.yaw);
  odom.y += fwd * std::sin(odom.yaw) + left * std::cos(odom.yaw);
}
}  // namespace synthetic_hall

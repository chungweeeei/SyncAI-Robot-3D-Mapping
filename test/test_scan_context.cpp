#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "syncai_mapping/pgos/scan_context.h"

namespace
{
// A sensor-centred, levelled "room": a floor, four walls and a few boxes of
// different heights, so the descriptor has structure in every direction.
CloudType makeRoom(unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  CloudType cloud;
  auto add = [&cloud](float x, float y, float z) {
    PointType p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 0.0f;
    cloud.push_back(p);
  };
  for (float x = -12.0f; x <= 12.0f; x += 0.2f)
    for (float y = -7.0f; y <= 7.0f; y += 0.2f) add(x, y, -0.5f);
  for (float z = -0.5f; z <= 2.5f; z += 0.2f) {
    for (float x = -12.0f; x <= 12.0f; x += 0.2f) {
      add(x, -7.0f, z);
      add(x, 7.0f, z);
    }
    for (float y = -7.0f; y <= 7.0f; y += 0.2f) {
      add(-12.0f, y, z);
      add(12.0f, y, z);
    }
  }
  for (int b = 0; b < 8; b++) {
    const float cx = 10.0f * u(rng), cy = 5.0f * u(rng), h = 1.5f + u(rng);
    for (float z = -0.5f; z <= h; z += 0.1f)
      for (float a = -0.4f; a <= 0.4f; a += 0.1f) {
        add(cx + a, cy - 0.4f, z);
        add(cx + a, cy + 0.4f, z);
        add(cx - 0.4f, cy + a, z);
        add(cx + 0.4f, cy + a, z);
      }
  }
  return cloud;
}

CloudType rotated(const CloudType & in, double yaw)
{
  CloudType out = in;
  const float c = std::cos(yaw), s = std::sin(yaw);
  for (PointType & p : out.points) {
    const float x = p.x, y = p.y;
    p.x = c * x - s * y;
    p.y = s * x + c * y;
  }
  return out;
}
}  // namespace

TEST(ScanContext, RecoversYawOfTheSamePlace)
{
  const CloudType room = makeRoom(1);
  const ScanContext::Descriptor candidate = ScanContext::make(room, 20.0);
  for (double psi_deg : {36.0, -60.0, 150.0}) {
    // The query frame is the candidate's rotated by psi: a point fixed in the
    // world appears rotated by -psi in it.
    const double psi = psi_deg * M_PI / 180.0;
    const ScanContext::Descriptor query = ScanContext::make(rotated(room, -psi), 20.0);
    int shift = -1;
    const double dist = ScanContext::distance(query, candidate, shift);
    EXPECT_LT(dist, 0.05) << "psi " << psi_deg;
    EXPECT_NEAR(ScanContext::yawOfShift(shift), psi, 1e-6) << "psi " << psi_deg;
    // The ring key ignores rotation.
    EXPECT_LT(
      (ScanContext::ringKey(query) - ScanContext::ringKey(candidate)).norm(),
      0.05 * ScanContext::ringKey(candidate).norm());
  }
}

TEST(ScanContext, DifferentPlacesAreFarApart)
{
  // The same room seen from 6 m further along: a different place in the same
  // building. (Two rooms that differ only in their boxes score ~0.04 -- the
  // walls dominate; that aliasing is what the drift gate is for.)
  CloudType moved = makeRoom(1);
  for (PointType & p : moved.points) {
    p.x -= 6.0f;
    p.y -= 3.0f;
  }
  const ScanContext::Descriptor a = ScanContext::make(makeRoom(1), 20.0);
  const ScanContext::Descriptor b = ScanContext::make(moved, 20.0);
  int shift = 0;
  const double same = ScanContext::distance(a, a, shift);
  const double other = ScanContext::distance(a, b, shift);
  EXPECT_NEAR(same, 0.0, 1e-6);
  EXPECT_GT(other, same + 0.05);
}

TEST(ScanContext, EmptyCloudIsNotComparable)
{
  const ScanContext::Descriptor empty = ScanContext::make(CloudType(), 20.0);
  const ScanContext::Descriptor room = ScanContext::make(makeRoom(1), 20.0);
  int shift = 0;
  EXPECT_DOUBLE_EQ(ScanContext::distance(empty, room, shift), 1.0);
}

#include "syncai_mapping/pgos/scan_context.h"

#include <cmath>

ScanContext::Descriptor ScanContext::make(const CloudType & level_cloud, double max_range)
{
  Descriptor desc = Descriptor::Zero(kNumRings, kNumSectors);
  const double ring_width = max_range / kNumRings;
  const double sector_width = 2.0 * M_PI / kNumSectors;
  for (const PointType & p : level_cloud.points) {
    const double range = std::hypot(p.x, p.y);
    if (range >= max_range || range < 1e-3) continue;
    double azimuth = std::atan2(p.y, p.x);
    if (azimuth < 0.0) azimuth += 2.0 * M_PI;
    const int ring = std::min(static_cast<int>(range / ring_width), kNumRings - 1);
    const int sector = std::min(static_cast<int>(azimuth / sector_width), kNumSectors - 1);
    // Points below the floor (reflections) would go negative; clamp so they
    // never mark a bin as occupied by "less than nothing".
    const float height = std::max(p.z + kHeightOffset, 0.0f);
    desc(ring, sector) = std::max(desc(ring, sector), height);
  }
  return desc;
}

Eigen::VectorXf ScanContext::ringKey(const Descriptor & desc)
{
  return desc.rowwise().mean();
}

double ScanContext::distance(
  const Descriptor & query, const Descriptor & candidate, int & best_shift)
{
  // Column norms once; a column is comparable only when both sides have one.
  const Eigen::RowVectorXf q_norm = query.colwise().norm();
  const Eigen::RowVectorXf c_norm = candidate.colwise().norm();

  double best = 1.0;
  best_shift = 0;
  for (int shift = 0; shift < kNumSectors; shift++) {
    double sim_sum = 0.0;
    int count = 0;
    for (int j = 0; j < kNumSectors; j++) {
      const int k = (j + shift) % kNumSectors;
      if (q_norm(j) <= 0.0f || c_norm(k) <= 0.0f) continue;
      sim_sum += query.col(j).dot(candidate.col(k)) / (q_norm(j) * c_norm(k));
      count++;
    }
    if (count == 0) continue;
    const double dist = 1.0 - sim_sum / count;
    if (dist < best) {
      best = dist;
      best_shift = shift;
    }
  }
  return best;
}

double ScanContext::yawOfShift(int shift)
{
  double yaw = shift * 2.0 * M_PI / kNumSectors;
  if (yaw > M_PI) yaw -= 2.0 * M_PI;
  return yaw;
}

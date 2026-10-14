#pragma once
#include <Eigen/Core>

#include "syncai_mapping/pgos/commons.h"

/**
 * Scan Context (Kim & Kim, IROS 2018): a pose-independent place descriptor, the global source of
 * loop candidates. Unlike the radius search it does not trust the current estimate, so it still
 * finds a revisit after drift beyond loop_search_radius.
 *
 * Descriptor: num_rings x num_sectors polar grid around the sensor holding each bin's highest
 * point (z + kHeightOffset; 0 = empty). The cloud must be gravity-levelled and sensor-centred: the
 * lidar is pitched ~15 deg, so a raw body cloud would describe the mounting, not the place.
 *
 * Yaw convention: if the query's level frame is the candidate's rotated by psi about z, query
 * sector j matches candidate sector j + psi / sector_width. distance() returns that shift and
 * yawOfShift() turns it into psi.
 */
class ScanContext
{
public:
  using Descriptor = Eigen::MatrixXf;  // num_rings x num_sectors

  static constexpr int kNumRings = 20;
  static constexpr int kNumSectors = 60;
  // Keeps floor bins (z ~ -lidar height) > 0, distinct from empty (0). The original's LIDAR_HEIGHT.
  static constexpr float kHeightOffset = 2.0f;

  // Descriptor of a gravity-levelled, sensor-centred cloud; points beyond max_range ignored.
  static Descriptor make(const CloudType & level_cloud, double max_range);

  // Mean of each ring: rotation-invariant, used to preselect candidates.
  static Eigen::VectorXf ringKey(const Descriptor & desc);

  // Column-wise cosine distance in [0, 2] (0 = identical) at the best sector shift, written to
  // best_shift. Columns empty on either side are skipped; 1.0 when none is comparable.
  static double distance(const Descriptor & query, const Descriptor & candidate, int & best_shift);

  // psi in (-pi, pi] for a shift returned by distance().
  static double yawOfShift(int shift);
};

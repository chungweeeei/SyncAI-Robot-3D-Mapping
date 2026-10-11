#pragma once
#include <Eigen/Core>

#include "syncai_mapping/pgos/commons.h"

/**
 * Scan Context (Kim & Kim, IROS 2018): a pose-independent place descriptor,
 * used as the second, global source of loop candidates. The radius search
 * only finds a revisit when the drift is below loop_search_radius; Scan
 * Context compares the scans themselves, so it still finds one after metres
 * of drift. It plays the part of Cartographer's global constraint search,
 * which also looks for loops without trusting the current pose estimate.
 *
 * The descriptor is a num_rings x num_sectors polar grid around the sensor
 * holding, per bin, the highest point (z + kHeightOffset, so the floor is
 * > 0; 0 = empty bin). The cloud must be gravity-levelled (no roll / pitch)
 * and centred on the sensor: the lidar is pitched ~15 deg on this robot, so
 * a raw body cloud would describe the mounting, not the place.
 *
 * Yaw convention: if the query's level frame is the candidate's rotated by
 * psi about z, a query point at azimuth a sits at a + psi in the candidate,
 * so query sector j matches candidate sector j + psi / sector_width.
 * distance() returns that shift, and yawOfShift() turns it into psi.
 */
class ScanContext
{
public:
  using Descriptor = Eigen::MatrixXf;  // num_rings x num_sectors

  static constexpr int kNumRings = 20;
  static constexpr int kNumSectors = 60;
  // Added to every z so that bins on the floor (z ~ -lidar height) are still
  // > 0 and an empty bin (0) stays distinguishable. Same constant as the
  // original implementation (its LIDAR_HEIGHT).
  static constexpr float kHeightOffset = 2.0f;

  // Builds the descriptor of a gravity-levelled, sensor-centred cloud.
  // Points beyond max_range are ignored.
  static Descriptor make(const CloudType & level_cloud, double max_range);

  // Mean of each ring: rotation-invariant, used to preselect candidates.
  static Eigen::VectorXf ringKey(const Descriptor & desc);

  // Column-wise cosine distance in [0, 2] (0 = identical) at the best sector
  // shift, which is written to best_shift (query sector j <-> candidate
  // sector j + best_shift). Columns empty in either descriptor are skipped;
  // 1.0 when no column pair is comparable.
  static double distance(const Descriptor & query, const Descriptor & candidate, int & best_shift);

  // psi in (-pi, pi] for a shift returned by distance().
  static double yawOfShift(int shift);
};

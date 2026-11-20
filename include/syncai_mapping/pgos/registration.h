#pragma once
#include <small_gicp/pcl/pcl_registration.hpp>

#include "syncai_mapping/pgos/commons.h"

// Registering one cloud onto another and judging the result: the part of loop verification that
// does not know what a keyframe is. PoseGraphOptimizer::verifyCandidate wraps it with the graph's
// own gate (drift against the path travelled); SessionAligner uses it to place a resumed run in
// its saved map, where there is no graph yet.

using GicpRegistration = small_gicp::RegistrationPCL<PointType, PointType>;

// The coarse stage relative to the fine one: 3x voxel and 3x correspondence distance, so a ~3 m
// offset converges.
inline constexpr double kCoarseFactor = 3.0;

M3D yawRotation(double yaw);
double yawOf(const M3D & r);
M4D toMatrix(const M3D & r, const V3D & t);
CloudType::Ptr voxelized(const CloudType::Ptr & cloud, double resolution);

// Same settings as syncai_localizer's refine stage. setNumThreads() must precede the first
// setInputTarget() (the kd-tree builder reads it), so this is called once, on a new object.
void configureGicp(
  GicpRegistration & gicp, int num_threads, int num_neighbors, double max_corr_dist);

// See PoseGraphOptimizerConfig, whose loop_* fields these are.
struct RegistrationGates
{
  double resolution = 0.1;  // of source and target; the coarse stage voxelises both 3x coarser
  double max_corr_dist = 1.0;
  double inlier_dist = 0.2;
  double min_overlap_ratio = 0.3;
  double min_inlier_ratio = 0.6;
  double score_tresh = 0.15;
  double min_xy_info_ratio = 0.0;
  // Whether a stage that ran out of iterations is a rejection. GICP's stop test is a step under
  // a micrometre, which a registration can circle above for all 50 iterations while sitting on
  // the answer. A loop takes that as a rejection, as it always has: the next keyframe brings
  // another candidate. The session alignment cannot afford to lose the right candidate that way
  // (the look-alike test needs it among the passed), so there the gates below judge whatever the
  // last iteration left.
  bool require_convergence = true;
};

struct RegistrationReport
{
  // The fine stage met its stop test. Always true of a registration that passed with
  // require_convergence.
  bool converged = false;
  // A transform was produced and judged (the figures below are of it).
  bool registered = false;
  double fitness = 0.0;
  double overlap_ratio = 0.0;
  double inlier_ratio = 0.0;
  double xy_info_ratio = 0.0;
  // Inliers over ALL source points (inlier_ratio is over the overlap only): what a scan that
  // overlaps the target in one corner cannot game.
  double source_inlier_ratio = 0.0;
  const char * reject_reason = "";
};

/**
 * GICP of `source` (already at its initial pose in the target's frame, voxelised) onto `target`,
 * then the overlap / inlier / fitness / degeneracy gates. With `coarse_first`, a coarse GICP on
 * 3x coarser clouds seeds the fine one: for a guess that may be metres off, beyond the fine
 * stage's correspondence distance. `coarse` must have been configured with 3x `fine`'s
 * correspondence distance.
 *
 * True when every gate passed; `transform` then moves the source's frame onto the target (its
 * registered pose = transform * initial pose). The report is filled either way.
 */
bool registerAndScore(
  GicpRegistration & fine, GicpRegistration & coarse, const CloudType::Ptr & source,
  const CloudType::Ptr & target, bool coarse_first, const RegistrationGates & gates,
  RegistrationReport & report, M4D & transform);

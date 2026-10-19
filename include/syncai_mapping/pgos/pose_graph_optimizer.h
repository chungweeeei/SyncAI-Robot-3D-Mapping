#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <small_gicp/pcl/pcl_registration.hpp>

#include <string>
#include <vector>

#include "syncai_mapping/pgos/commons.h"
#include "syncai_mapping/pgos/gravity_prior_factor.h"
#include "syncai_mapping/pgos/planar_loop_factor.h"
#include "syncai_mapping/pgos/scan_context.h"

struct KeyPoseWithCloud
{
  M3D r_local;
  V3D t_local;
  M3D r_global;
  V3D t_global;
  double time;
  // Path length (m) from the first keyframe, along the LIO poses.
  double travelled;
  CloudType::Ptr body_cloud;
  // Only with loop_sc_enabled; built once at insertion from the LIO tilt.
  ScanContext::Descriptor sc_desc;
  Eigen::VectorXf sc_ring_key;
};
struct LoopPair
{
  size_t source_id;
  size_t target_id;
  M3D r_offset;
  V3D t_offset;
  double score;
  // World-frame correction behind r_offset / t_offset (the GICP result, projected to x / y / yaw
  // with loop_planar_correction): moves the source keyframe onto the target submap. Kept for
  // pgo_node's log (notably its z).
  M3D gicp_r;
  V3D gicp_t;
  // (dx, dy, dyaw) of the corrected source relative to the target, world frame: the
  // PlanarLoopFactor measurement. r_offset / t_offset feed the 6-DOF BetweenFactor instead.
  V3D planar_meas;
};

struct PoseGraphOptimizerConfig
{
  double keyframe_delta_deg = 10;
  double keyframe_delta_m = 1.0;
  double loop_search_radius = 1.0;
  double loop_time_tresh = 60.0;
  double loop_score_tresh = 0.15;
  // Loops are verified with small_gicp GICP; the fork's PCL point-to-point ICP returned
  // rotation and z several times too large on corridor loops.
  //
  // Max GICP correspondence distance (m). The fork's 10 effectively disabled the gate: far points
  // of a long corridor scan paired with whatever was nearest and steered the solution.
  double loop_gicp_max_corr_dist = 1.0;
  int loop_gicp_num_threads = 4;
  int loop_gicp_num_neighbors = 20;
  // Constrain only world x / y / yaw (PlanarLoopFactor) instead of a 6-DOF BetweenFactor; the
  // registration is projected onto x / y / yaw about world z first. Point-LIO's world frame is
  // gravity-aligned, so a loop has nothing to add on z / roll / pitch, while a 6-DOF edge with a
  // slightly wrong tilt bends the chain into a double floor (and a body-frame edge leaks x into z
  // through the 15 deg lidar pitch). Off = the fork's 6-DOF behaviour.
  bool loop_planar_correction = true;
  // Only with loop_planar_correction = false: BetweenFactor variance on roll / pitch / z
  // (rad^2 / m^2); x / y / yaw keep the fork's "variance = registration fitness".
  double loop_noise_var_roll_pitch_z = 1e-2;
  // Only with loop_planar_correction = true: yaw sigma (deg). The fork's "variance = fitness"
  // read as rad^2 is a ~20 deg sigma, which the odometry chain (~1 deg accumulated) outvotes, so
  // loops never corrected yaw. <= 0 = variance = fitness.
  double loop_noise_yaw_sigma_deg = 1.5;
  // Only with loop_planar_correction = true: x / y sigma (m). "Variance = fitness" is a
  // 0.25-0.39 m sigma at typical corridor fitness, which a few hundred odometry edges outvote,
  // leaving loops half-closed. <= 0 = variance = fitness.
  double loop_noise_xy_sigma_m = 0.05;
  // Per-keyframe tilt anchor (GravityPriorFactor) sigma (deg), holding roll / pitch to the LIO.
  // Needed once loops correct yaw with real weight, or yaw is partly realised as pitch and the
  // chain climbs or sinks. <= 0 = no anchor.
  double keyframe_tilt_sigma_deg = 0.5;
  int loop_submap_half_range = 5;
  double loop_submap_resolution = 0.1;
  double loop_search_period = 10.0;

  // Robust kernel on every loop factor: "huber", "cauchy" or "none", so one wrong loop that
  // passed verification cannot pull the chain with a force growing with its error. The scale is
  // in whitened units (sigmas of the loop noise); below it, plain least squares. It must sit
  // above where a correct loop settles (~3 sigma for a large yaw correction), so 5.
  std::string loop_robust_kernel = "huber";
  double loop_robust_kernel_scale = 5.0;

  // Verification after GICP, aligned source vs target submap. A source point is in the OVERLAP
  // when the submap has a point within loop_gicp_max_corr_dist, and an INLIER within
  // loop_inlier_dist. Gates, so the part of the scan the submap never saw does not count against
  // a good loop:
  //  - overlap ratio (overlap / source) >= loop_min_overlap_ratio;
  //  - inlier ratio (inliers / overlap) >= loop_min_inlier_ratio;
  //  - fitness = mean squared distance over the overlap <= loop_score_tresh.
  double loop_inlier_dist = 0.2;
  double loop_min_overlap_ratio = 0.3;
  double loop_min_inlier_ratio = 0.6;
  // Corridor degeneracy gate: smallest / largest eigenvalue of the GICP Hessian's world x / y
  // translation block; a straight corridor lets a loop slide along it with perfect fitness.
  // <= 0 = off (still logged, to calibrate).
  double loop_min_xy_info_ratio = 0.0;
  // Drift plausibility gate: the xy correction must not exceed this fraction of the path travelled
  // between the two keyframes (floored at loop_search_radius). Point-LIO drifts well under 1 %,
  // while a look-alike place (symmetric corridor, far side of a hall) asks for a jump of its
  // separation. Matters most for Scan Context, whose candidates ignore the estimate. <= 0 = off.
  double loop_max_drift_ratio = 0.05;
  // Radius search reach (xy): false = loop_search_radius only, true = per candidate the drift
  // gate's bound max(loop_search_radius, loop_max_drift_ratio * path), capped at
  // loop_search_radius + the coarse stage's reach (3 * loop_gicp_max_corr_dist). Candidates
  // beyond loop_search_radius are registered coarse to fine. z is held to loop_search_radius
  // either way. Without it, a revisit whose drift exceeds loop_search_radius is never a
  // candidate, however much the drift gate would allow.
  bool loop_radius_drift_scaled = true;
  // Radius candidates verified per search (nearest first, at least one submap apart); the best
  // inlier ratio wins. 1 = the fork's "first candidate only".
  int loop_max_candidates = 3;

  // Scan Context, the global candidate source (see scan_context.h), tried only when the radius
  // search accepted nothing. Aligned coarse to fine: GICP on a 3x coarser submap with a 3x
  // correspondence distance, then the normal verification.
  bool loop_sc_enabled = false;
  double loop_sc_max_range = 20.0;  // m, descriptor radius
  double loop_sc_dist_tresh = 0.2;  // cosine distance, lower = stricter
};

// One loop candidate's outcome, for pgo_node's log (filled for every candidate, to calibrate).
struct LoopCandidateReport
{
  size_t target_id = 0;
  size_t source_id = 0;
  bool scan_context = false;  // false = radius search
  bool coarse = false;        // registered coarse to fine (Scan Context, or a far radius one)
  double sc_dist = 0.0;       // only with scan_context
  bool converged = false;
  double fitness = 0.0;
  double overlap_ratio = 0.0;
  double inlier_ratio = 0.0;
  double xy_info_ratio = 0.0;
  double correction_m = 0.0;  // xy distance the loop moves the current keyframe
  double allowed_m = 0.0;     // what the drift gate allowed
  bool accepted = false;      // passed every gate (the best one becomes the loop)
  const char * reject_reason = "";
};

class PoseGraphOptimizer
{
public:
  PoseGraphOptimizer(const PoseGraphOptimizerConfig & config);

  bool isKeyPose(const PoseWithTime & pose);

  bool addKeyPose(const CloudWithPose & cloud_with_pose);

  bool hasLoop() { return m_cache_pairs.size() > 0; }
  // Loops accepted by the last searchForLoopPairs(); valid until smoothAndUpdate() consumes them.
  const std::vector<LoopPair> & pendingLoops() const { return m_cache_pairs; }

  void searchForLoopPairs();

  void smoothAndUpdate();

  CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
  // Every candidate the last searchForLoopPairs() verified, accepted or not.
  const std::vector<LoopCandidateReport> & lastCandidates() const { return m_last_candidates; }
  // Extra iSAM2 update() steps the last smoothAndUpdate() took to settle a loop (0 without one).
  int lastLoopIterations() const { return m_last_loop_iterations; }
  std::vector<std::pair<size_t, size_t>> & historyPairs() { return m_history_pairs; }
  std::vector<KeyPoseWithCloud> & keyPoses() { return m_key_poses; }

  M3D offsetR() { return m_r_offset; }
  V3D offsetT() { return m_t_offset; }

private:
  // A keyframe to verify the current one against, and the initial pose of the current body cloud
  // for GICP (its world pose for a radius candidate; a Scan Context yaw guess for a global one).
  struct LoopCandidate
  {
    size_t target_id;
    M3D r_init;
    V3D t_init;
    bool scan_context;
    bool coarse;  // run the coarse GICP stage first (the guess may be metres off)
    double sc_dist;
  };

  std::vector<LoopCandidate> radiusCandidates(size_t cur_idx);
  bool scanContextCandidate(size_t cur_idx, LoopCandidate & candidate);
  // GICP plus the overlap / inlier / degeneracy gates. On success, the world-frame correction
  // from the current keyframe's graph pose to the registered one.
  bool verifyCandidate(
    size_t cur_idx, const LoopCandidate & candidate, LoopCandidateReport & report,
    M4D & correction);
  LoopPair makeLoopPair(size_t cur_idx, size_t loop_idx, const M4D & correction, double score);
  gtsam::SharedNoiseModel robustify(const gtsam::SharedNoiseModel & model) const;

  PoseGraphOptimizerConfig m_config;
  std::vector<KeyPoseWithCloud> m_key_poses;
  std::vector<std::pair<size_t, size_t>> m_history_pairs;
  std::vector<LoopPair> m_cache_pairs;
  M3D m_r_offset;
  V3D m_t_offset;
  std::shared_ptr<gtsam::ISAM2> m_isam2;
  gtsam::Values m_initial_values;
  gtsam::NonlinearFactorGraph m_graph;
  small_gicp::RegistrationPCL<PointType, PointType> m_gicp;
  // The coarse stage of a Scan Context candidate (see loop_sc_enabled).
  small_gicp::RegistrationPCL<PointType, PointType> m_gicp_coarse;
  std::vector<LoopCandidateReport> m_last_candidates;
  int m_last_loop_iterations = 0;
};

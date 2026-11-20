#pragma once
#include <string>
#include <vector>

#include "syncai_mapping/pgos/pose_graph_optimizer.h"
#include "syncai_mapping/pgos/registration.h"

/**
 * Where a new run stands in a saved map: the map <- odom transform resume_mapping needs before
 * its first keyframe, since nothing can seed Point-LIO with a pose (its odometry frame begins at
 * the robot, levelled, yaw 0, after every reset).
 *
 * The source is a short accumulation of scans in the odometry frame, taken while the robot
 * stands; the map is the saved keyframes themselves (PoseGraphOptimizer::loadPrior's input), not
 * map.pcd. One pipeline, two candidate sources:
 *
 *  - HINT: the caller's rough pose. Candidates are the hint and the saved keyframe positions
 *    within hint_radius of it, over the yaws within hint_yaw_deg of the hint's.
 *  - GLOBAL: every saved keyframe position (thinned to candidate_spacing) over a full yaw sweep,
 *    the kidnap search syncai_localizer runs in the nav session. Re-written here because that
 *    package has no library to link, and the mapping session does not run it.
 *
 * Every candidate is scored cheaply (the share of the scan that lands on an occupied cell of the
 * saved map), the best few distinct ones are registered coarse to fine (registration.h) against
 * the saved keyframes AROUND them -- by distance, not by index as a loop's submap is: the robot
 * may stand metres off the old track, and the place may have been passed several times -- and
 * the best registration that passes its gates wins.
 *
 * A refusal is the point of doing this before the run rather than trusting the first loop.
 * GLOBAL only: a second passing registration somewhere else, scoring within ambiguity_ratio of
 * the best, is a place that looks like another one. Refused, never guessed; a hint resolves it.
 * A hint is not tested this way (the caller has said which one it is), and a hint that fails is
 * not retried globally: it usually means the operator and the robot disagree.
 *
 * THE REGISTRATION'S ROLL / PITCH IS KEPT: the result is the whole map <- odom transform, not a
 * rotation about z. Both frames are gravity-aligned, but each by its own static IMU
 * initialisation, and those do not agree: on robot01 two runs begun 85 deg apart in heading
 * levelled themselves 0.76 deg apart (2026-11-20, record/resume_a + resume_b), where two begun
 * at one pose differed by 0.08 -- an error of about half a degree that turns with the sensor, an
 * accelerometer bias by its signature. Dropped, as this did at first, that tilt left the new run
 * leaning on the saved map: 1 cm a metre, 5.7 cm within 6 m of the start. The saved map is the
 * one that cannot move, so its up is the run's (PoseGraphOptimizer::loadPrior takes it from this
 * transform). The tilt is reported, and above max_tilt_deg the alignment fails: by then the
 * registration is more likely wrong than the levelling that far off.
 *
 * ROS-free. Every threshold here is a first guess, to be calibrated on replays from the attempts
 * the result lists.
 */
struct SessionAlignerConfig
{
  int gicp_num_threads = 4;
  int gicp_num_neighbors = 20;
  // As a loop's, except min_inlier_ratio (0.75, set below): a loop that passes wrongly is one
  // edge under a robust kernel, an alignment that passes wrongly is the whole run in the wrong
  // place. require_convergence is always off here, whatever is set (see RegistrationGates).
  RegistrationGates gates;
  SessionAlignerConfig() { gates.min_inlier_ratio = 0.75; }
  // Inliers over the whole scan. Low: starting at the saved map's edge, where half the scan
  // sees what was never mapped, is what resuming is for.
  double min_source_inlier_ratio = 0.3;
  // Saved keyframes within this of a candidate (xy, and z within target_z_band) are its
  // registration target.
  double target_radius = 12.0;
  // Also the storey a hint's z selects.
  double target_z_band = 1.5;
  // The most the two runs' ups may differ (see above). Not a quality gate on the levelling, which
  // is corrected whatever it is: a sanity bound on the registration.
  double max_tilt_deg = 3.0;

  double hint_radius = 3.0;
  double hint_yaw_deg = 45.0;

  double candidate_spacing = 1.0;
  double yaw_step_deg = 15.0;
  // Registrations per alignment (the distinct best-scoring candidates).
  int max_registrations = 10;
  double ambiguity_ratio = 0.9;
  // Wall time for the registrations. What has passed by then is judged; nothing passed is a
  // failure.
  double timeout_sec = 10.0;
};

struct SessionAlignRequest
{
  // The accumulated scan, in the odometry frame.
  CloudType::Ptr cloud_odom;
  // The body in the odometry frame when the scan was taken (its last pose).
  M3D r_body = M3D::Identity();
  V3D t_body = V3D::Zero();
  // The body's rough pose in the map. z only picks the storey.
  bool use_hint = false;
  V3D hint_position = V3D::Zero();
  double hint_yaw = 0.0;
};

// One registered candidate, for the log.
struct SessionAlignAttempt
{
  V3D position = V3D::Zero();  // the body in the map: the candidate, or as registered when it
  double yaw = 0.0;            // converged
  double coarse_score = 0.0;
  RegistrationReport registration;
  double tilt_residual_deg = 0.0;
  bool passed = false;
  const char * reject_reason = "";
};

struct SessionAlignResult
{
  bool success = false;
  // On failure, a sentence fit for the operator.
  std::string message;
  bool used_hint = false;
  // map <- odom, as registered: mostly a rotation about world z, plus the tilt between the two
  // runs' ups.
  M3D r_offset = M3D::Identity();
  V3D t_offset = V3D::Zero();
  // The body in the map.
  V3D position = V3D::Zero();
  double yaw = 0.0;
  RegistrationReport registration;
  // The part of r_offset that is not a rotation about z (deg).
  double tilt_residual_deg = 0.0;
  // Candidates scored, and those registered (best coarse score first).
  size_t candidates = 0;
  std::vector<SessionAlignAttempt> attempts;
};

class SessionAligner
{
public:
  explicit SessionAligner(const SessionAlignerConfig & config);

  // `prior`: the saved keyframes, global poses and body clouds (preview clouds, where present,
  // are used for the coarse map).
  SessionAlignResult align(
    const std::vector<KeyPoseWithCloud> & prior, const SessionAlignRequest & request);

private:
  SessionAlignerConfig m_config;
  // Its own pair, as GlobalLocalizer has: the optimizer's is being built on another thread.
  GicpRegistration m_gicp;
  GicpRegistration m_gicp_coarse;
};

#include "syncai_mapping/pgos/pose_graph_optimizer.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace
{
// Scan Context: candidates kept by the ring-key preselection.
constexpr size_t kScRingKeyCandidates = 10;
// The coarse stage (Scan Context candidates, and radius candidates beyond loop_search_radius),
// relative to the fine one: 3x voxel and 3x correspondence distance, so a ~3 m offset converges.
constexpr double kCoarseFactor = 3.0;
// After a loop, smoothAndUpdate runs at least kMinLoopIterations extra iSAM2 steps (upstream's
// fixed count), then until the newest keyframe moves less than kLoopSettledTangent (tangent norm,
// rad and m) per step, at most kMaxLoopIterations.
constexpr int kMinLoopIterations = 4;
constexpr int kMaxLoopIterations = 20;
constexpr double kLoopSettledTangent = 1e-5;

M3D yawRotation(double yaw)
{
  return Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();
}
double yawOf(const M3D & r)
{
  return std::atan2(r(1, 0), r(0, 0));
}

M4D toMatrix(const M3D & r, const V3D & t)
{
  M4D m = M4D::Identity();
  m.block<3, 3>(0, 0) = r;
  m.block<3, 1>(0, 3) = t;
  return m;
}

CloudType::Ptr voxelized(const CloudType::Ptr & cloud, double resolution)
{
  CloudType::Ptr ret(new CloudType);
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(resolution, resolution, resolution);
  voxel_grid.setInputCloud(cloud);
  voxel_grid.filter(*ret);
  return ret;
}
}  // namespace

PoseGraphOptimizer::PoseGraphOptimizer(const PoseGraphOptimizerConfig & config) : m_config(config)
{
  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeThreshold = 0.01;
  isam2_params.relinearizeSkip = 1;
  m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
  m_initial_values.clear();
  m_graph.resize(0);
  m_r_offset.setIdentity();
  m_t_offset.setZero();

  // Same settings as syncai_localizer's refine stage. setNumThreads() must precede the first
  // setInputTarget() (the kd-tree builder reads it).
  m_gicp.setNumThreads(m_config.loop_gicp_num_threads);
  m_gicp.setCorrespondenceRandomness(m_config.loop_gicp_num_neighbors);
  m_gicp.setRegistrationType("GICP");
  m_gicp.setMaximumIterations(50);
  m_gicp.setMaxCorrespondenceDistance(m_config.loop_gicp_max_corr_dist);
  m_gicp.setTransformationEpsilon(1e-6);

  m_gicp_coarse.setNumThreads(m_config.loop_gicp_num_threads);
  m_gicp_coarse.setCorrespondenceRandomness(m_config.loop_gicp_num_neighbors);
  m_gicp_coarse.setRegistrationType("GICP");
  m_gicp_coarse.setMaximumIterations(50);
  m_gicp_coarse.setMaxCorrespondenceDistance(kCoarseFactor * m_config.loop_gicp_max_corr_dist);
  m_gicp_coarse.setTransformationEpsilon(1e-6);
}

bool PoseGraphOptimizer::isKeyPose(const PoseWithTime & pose)
{
  if (m_key_poses.size() == 0) return true;

  const KeyPoseWithCloud & last_item = m_key_poses.back();

  double delta_trans = (pose.t - last_item.t_local).norm();
  double delta_deg =
    Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;

  if (delta_trans > m_config.keyframe_delta_m || delta_deg > m_config.keyframe_delta_deg)
    return true;

  return false;
}

bool PoseGraphOptimizer::addKeyPose(const CloudWithPose & cloud_with_pose)
{
  bool is_key_pose = isKeyPose(cloud_with_pose.pose);
  if (!is_key_pose) return false;

  size_t idx = m_key_poses.size();  // Index of the new node

  // New keyframes are seeded in the global frame through the last local -> global offset, so
  // the initial value is consistent with the already optimised graph.
  M3D init_r = m_r_offset * cloud_with_pose.pose.r;
  V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;

  m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));

  if (idx == 0) {
    // Node 0 is anchored absolutely.
    gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
    m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
  } else {
    const KeyPoseWithCloud & last_item = m_key_poses.back();

    M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
    V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);

    // Variances (roll, pitch, yaw, x, y, z): sigmas ~0.06 deg, ~1 cm in x / y, ~1 mm in z. The
    // short-range LIO relative pose is trusted strongly, z most of all.
    gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
    m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
  }

  // Tilt anchor (gravity_prior_factor.h): world up in the body frame, from the LOCAL (LIO)
  // rotation, which is gravity-aligned by construction, not from init_r. Global z equals local z
  // (node 0's prior is its local pose and loops only rotate about z), so the two ups agree.
  if (m_config.keyframe_tilt_sigma_deg > 0.0) {
    const double s = m_config.keyframe_tilt_sigma_deg * M_PI / 180.0;
    const V3D up = cloud_with_pose.pose.r.transpose() * V3D::UnitZ();
    m_graph.add(GravityPriorFactor(
      idx, gtsam::Vector3(up.x(), up.y(), up.z()), gtsam::noiseModel::Isotropic::Sigma(3, s)));
  }

  KeyPoseWithCloud item;
  item.time = cloud_with_pose.pose.second;
  item.r_local = cloud_with_pose.pose.r;
  item.t_local = cloud_with_pose.pose.t;
  item.body_cloud = cloud_with_pose.cloud;
  item.r_global = init_r;
  item.t_global = init_t;
  item.travelled = idx == 0 ? 0.0
                            : m_key_poses.back().travelled +
                                (cloud_with_pose.pose.t - m_key_poses.back().t_local).norm();
  if (m_config.loop_sc_enabled) {
    // Level the scan with the LIO tilt, keeping its yaw: the descriptor then depends on place and
    // heading only, and distance() recovers the heading.
    const M3D & r = cloud_with_pose.pose.r;
    const M3D tilt = yawRotation(-yawOf(r)) * r;
    CloudType level;
    pcl::transformPointCloud(
      *cloud_with_pose.cloud, level, V3D::Zero().eval(), Eigen::Quaterniond(tilt));
    item.sc_desc = ScanContext::make(level, m_config.loop_sc_max_range);
    item.sc_ring_key = ScanContext::ringKey(item.sc_desc);
  }
  m_key_poses.push_back(item);
  return true;
}

CloudType::Ptr PoseGraphOptimizer::getSubMap(int idx, int half_range, double resolution)
{
  assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
  int min_idx = std::max(0, idx - half_range);
  int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

  CloudType::Ptr ret(new CloudType);
  for (int i = min_idx; i <= max_idx; i++) {
    CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
    CloudType::Ptr global_cloud(new CloudType);
    pcl::transformPointCloud(
      *body_cloud, *global_cloud, m_key_poses[i].t_global,
      Eigen::Quaterniond(m_key_poses[i].r_global));
    *ret += *global_cloud;
  }
  if (resolution > 0) {
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setLeafSize(resolution, resolution, resolution);
    voxel_grid.setInputCloud(ret);
    voxel_grid.filter(*ret);
  }
  return ret;
}

std::vector<PoseGraphOptimizer::LoopCandidate> PoseGraphOptimizer::radiusCandidates(size_t cur_idx)
{
  const KeyPoseWithCloud & cur = m_key_poses[cur_idx];

  // How far (xy) a past keyframe may lie from the current estimate. A fixed loop_search_radius
  // never sees a revisit whose drift exceeds it, although the drift gate would accept a
  // correction of up to loop_max_drift_ratio of the path: with the defaults, 1 m against 5 m
  // after 100 m. With loop_radius_drift_scaled the search uses the gate's own bound, per
  // candidate, capped at what the coarse GICP stage can still pull in from the current estimate
  // (the guess); beyond that it cannot converge and the candidate only costs a registration.
  // z stays at loop_search_radius: loops do not correct z (planar), so a revisit is on the same
  // level, and a floor above is not one.
  const bool scaled = m_config.loop_radius_drift_scaled && m_config.loop_max_drift_ratio > 0.0;
  const double cap = m_config.loop_search_radius + kCoarseFactor * m_config.loop_gicp_max_corr_dist;

  // A linear scan, not a KD-tree: a per-candidate radius does not fit one search, and a few
  // thousand keyframes cost nothing next to one GICP.
  std::vector<std::pair<double, size_t>> near;  // (xy distance, index)
  for (size_t i = 0; i < cur_idx; i++) {
    const KeyPoseWithCloud & kp = m_key_poses[i];
    if (std::abs(cur.time - kp.time) <= m_config.loop_time_tresh) continue;
    if (std::abs(kp.t_global.z() - cur.t_global.z()) > m_config.loop_search_radius) continue;
    double allowed = m_config.loop_search_radius;
    if (scaled)
      allowed = std::min(
        cap, std::max(allowed, m_config.loop_max_drift_ratio * (cur.travelled - kp.travelled)));
    const double d = (kp.t_global - cur.t_global).head<2>().norm();
    if (d <= allowed) near.emplace_back(d, i);
  }
  std::sort(near.begin(), near.end());

  // Nearest first, and more than a submap apart in index from any chosen candidate (closer ones
  // would verify against nearly the same submap).
  std::vector<LoopCandidate> ret;
  const int min_gap = 2 * m_config.loop_submap_half_range + 1;
  const size_t max_candidates = static_cast<size_t>(std::max(1, m_config.loop_max_candidates));
  for (const auto & [d, idx] : near) {
    bool near_chosen = false;
    for (const LoopCandidate & c : ret)
      if (std::abs(static_cast<int>(idx) - static_cast<int>(c.target_id)) < min_gap)
        near_chosen = true;
    if (near_chosen) continue;
    // Beyond loop_search_radius the drift can exceed the fine stage's correspondence distance.
    const bool coarse = d > m_config.loop_search_radius;
    ret.push_back({idx, cur.r_global, cur.t_global, false, coarse, 0.0});
    if (ret.size() >= max_candidates) break;
  }
  return ret;
}

bool PoseGraphOptimizer::scanContextCandidate(size_t cur_idx, LoopCandidate & candidate)
{
  const KeyPoseWithCloud & cur = m_key_poses[cur_idx];
  if (cur.sc_desc.size() == 0) return false;

  // Ring keys are rotation-invariant, a cheap preselection. With the drift gate on, only targets
  // it could pass are considered (the guess puts the current keyframe on the target, so the
  // correction is about their distance, plus the coarse stage's reach).
  const double reach = kCoarseFactor * m_config.loop_gicp_max_corr_dist;
  std::vector<std::pair<float, size_t>> by_key;
  for (size_t i = 0; i < cur_idx; i++) {
    const KeyPoseWithCloud & kp = m_key_poses[i];
    if (cur.time - kp.time <= m_config.loop_time_tresh) continue;
    if (m_config.loop_max_drift_ratio > 0.0) {
      const double allowed = std::max(
        m_config.loop_search_radius,
        m_config.loop_max_drift_ratio * (cur.travelled - kp.travelled));
      if ((kp.t_global - cur.t_global).head<2>().norm() > allowed + reach) continue;
    }
    by_key.emplace_back((kp.sc_ring_key - cur.sc_ring_key).squaredNorm(), i);
  }
  if (by_key.empty()) return false;
  const size_t n = std::min(kScRingKeyCandidates, by_key.size());
  std::partial_sort(by_key.begin(), by_key.begin() + n, by_key.end());

  double best_dist = 1e9;
  int best_shift = 0;
  size_t best_idx = 0;
  for (size_t k = 0; k < n; k++) {
    int shift = 0;
    const double dist =
      ScanContext::distance(cur.sc_desc, m_key_poses[by_key[k].second].sc_desc, shift);
    if (dist < best_dist) {
      best_dist = dist;
      best_shift = shift;
      best_idx = by_key[k].second;
    }
  }
  if (best_dist > m_config.loop_sc_dist_tresh) return false;

  // Initial guess: at the target's position, with the target's heading turned by the descriptor
  // yaw and the current tilt kept. The position error is left to the coarse GICP stage.
  const KeyPoseWithCloud & tgt = m_key_poses[best_idx];
  const M3D cur_tilt = yawRotation(-yawOf(cur.r_global)) * cur.r_global;
  candidate.target_id = best_idx;
  candidate.r_init =
    yawRotation(yawOf(tgt.r_global) + ScanContext::yawOfShift(best_shift)) * cur_tilt;
  candidate.t_init = tgt.t_global;
  candidate.scan_context = true;
  candidate.coarse = true;
  candidate.sc_dist = best_dist;
  return true;
}

bool PoseGraphOptimizer::verifyCandidate(
  size_t cur_idx, const LoopCandidate & candidate, LoopCandidateReport & report, M4D & correction)
{
  report.target_id = candidate.target_id;
  report.source_id = cur_idx;
  report.scan_context = candidate.scan_context;
  report.coarse = candidate.coarse;
  report.sc_dist = candidate.sc_dist;
  const KeyPoseWithCloud & cur = m_key_poses[cur_idx];

  // Target: the submap around the old keyframe in world poses, voxelised. Source: the current
  // scan at the candidate's initial pose.
  CloudType::Ptr target = getSubMap(
    candidate.target_id, m_config.loop_submap_half_range, m_config.loop_submap_resolution);
  CloudType::Ptr source(new CloudType);
  pcl::transformPointCloud(
    *cur.body_cloud, *source, candidate.t_init, Eigen::Quaterniond(candidate.r_init));
  if (m_config.loop_submap_resolution > 0)
    source = voxelized(source, m_config.loop_submap_resolution);

  M4F guess = M4F::Identity();
  CloudType::Ptr aligned(new CloudType);
  if (candidate.coarse) {
    // Coarse first: the guess can be metres off, beyond the fine stage's correspondence distance.
    const double coarse_res = kCoarseFactor * std::max(m_config.loop_submap_resolution, 0.05);
    m_gicp_coarse.setInputSource(voxelized(source, coarse_res));
    m_gicp_coarse.setInputTarget(voxelized(target, coarse_res));
    m_gicp_coarse.align(*aligned);
    if (!m_gicp_coarse.hasConverged()) {
      report.reject_reason = "coarse GICP did not converge";
      return false;
    }
    guess = m_gicp_coarse.getFinalTransformation();
  }

  m_gicp.setInputSource(source);
  m_gicp.setInputTarget(target);
  m_gicp.align(*aligned, guess);
  if (!m_gicp.hasConverged()) {
    report.reject_reason = "GICP did not converge";
    return false;
  }
  report.converged = true;
  const M4D transform = m_gicp.getFinalTransformation().cast<double>();

  // Overlap / inlier / fitness over the aligned source (see the config).
  pcl::KdTreeFLANN<PointType> tree;
  tree.setInputCloud(target);
  const double overlap_sq = m_config.loop_gicp_max_corr_dist * m_config.loop_gicp_max_corr_dist;
  const double inlier_sq = m_config.loop_inlier_dist * m_config.loop_inlier_dist;
  size_t n_overlap = 0;
  size_t n_inlier = 0;
  double sum_sq = 0.0;
  std::vector<int> nn_id(1);
  std::vector<float> nn_sq(1);
  for (const PointType & p : aligned->points) {
    if (tree.nearestKSearch(p, 1, nn_id, nn_sq) == 0 || nn_sq[0] > overlap_sq) continue;
    n_overlap++;
    sum_sq += nn_sq[0];
    if (nn_sq[0] <= inlier_sq) n_inlier++;
  }
  report.overlap_ratio = aligned->empty() ? 0.0 : static_cast<double>(n_overlap) / aligned->size();
  report.inlier_ratio = n_overlap == 0 ? 0.0 : static_cast<double>(n_inlier) / n_overlap;
  report.fitness = n_overlap == 0 ? 1e9 : sum_sq / n_overlap;

  // Hessian order (rx, ry, rz, tx, ty, tz), perturbing the already world-frame source: the
  // (tx, ty) block is world x / y.
  const Eigen::Matrix2d h_xy = m_gicp.getFinalHessian().block<2, 2>(3, 3);
  const Eigen::Vector2d ev = Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(h_xy).eigenvalues();
  report.xy_info_ratio = ev(1) > 0.0 ? std::max(ev(0), 0.0) / ev(1) : 0.0;

  if (report.overlap_ratio < m_config.loop_min_overlap_ratio) {
    report.reject_reason = "overlap ratio";
    return false;
  }
  if (report.inlier_ratio < m_config.loop_min_inlier_ratio) {
    report.reject_reason = "inlier ratio";
    return false;
  }
  if (report.fitness > m_config.loop_score_tresh) {
    report.reject_reason = "fitness";
    return false;
  }
  if (
    m_config.loop_min_xy_info_ratio > 0.0 &&
    report.xy_info_ratio < m_config.loop_min_xy_info_ratio) {
    report.reject_reason = "degenerate (xy info ratio)";
    return false;
  }

  // Registered pose = transform * initial pose; the correction maps the graph pose onto it.
  const M4D refined = transform * toMatrix(candidate.r_init, candidate.t_init);
  report.correction_m = (refined.block<2, 1>(0, 3) - cur.t_global.head<2>()).norm();
  report.allowed_m = std::max(
    m_config.loop_search_radius,
    m_config.loop_max_drift_ratio * (cur.travelled - m_key_poses[candidate.target_id].travelled));
  if (m_config.loop_max_drift_ratio > 0.0 && report.correction_m > report.allowed_m) {
    report.reject_reason = "drift (correction too large for the path travelled)";
    return false;
  }
  report.accepted = true;
  correction = refined * toMatrix(cur.r_global, cur.t_global).inverse();
  return true;
}

void PoseGraphOptimizer::searchForLoopPairs()
{
  m_last_candidates.clear();

  if (m_key_poses.size() < 10) return;

  if (m_config.loop_search_period > 0.0) {
    if (m_history_pairs.size() > 0) {
      double current_time = m_key_poses.back().time;
      double last_time = m_key_poses[m_history_pairs.back().second].time;
      if (current_time - last_time < m_config.loop_search_period) return;
    }
  }

  // Proximity alone is not trusted (look-alike places): every candidate is registered and the
  // best inlier ratio among those that pass wins.
  const size_t cur_idx = m_key_poses.size() - 1;
  bool found = false;
  size_t best_target = 0;
  double best_inlier = -1.0;
  double best_fitness = 0.0;
  M4D best_correction = M4D::Identity();
  auto verify = [&](const LoopCandidate & candidate) {
    LoopCandidateReport report;
    M4D correction;
    if (
      verifyCandidate(cur_idx, candidate, report, correction) &&
      report.inlier_ratio > best_inlier) {
      found = true;
      best_target = candidate.target_id;
      best_inlier = report.inlier_ratio;
      best_fitness = report.fitness;
      best_correction = correction;
    }
    m_last_candidates.push_back(report);
  };

  for (const LoopCandidate & candidate : radiusCandidates(cur_idx)) verify(candidate);
  if (!found && m_config.loop_sc_enabled) {
    LoopCandidate candidate;
    if (scanContextCandidate(cur_idx, candidate)) verify(candidate);
  }
  if (!found) return;

  m_cache_pairs.push_back(makeLoopPair(cur_idx, best_target, best_correction, best_fitness));

  // For the markers and the cool-down.
  m_history_pairs.emplace_back(best_target, cur_idx);
}

LoopPair PoseGraphOptimizer::makeLoopPair(
  size_t cur_idx, size_t loop_idx, const M4D & correction, double score)
{
  // World-frame correction mapping the source keyframe's world cloud onto the submap.
  M4D loop_transform = correction;
  if (m_config.loop_planar_correction) {
    // Keep only x / y / yaw; world z, roll and pitch stay as the graph has them. The source POSE
    // is displaced by the registration's xy displacement (the raw translation column is
    // meaningless far from the origin when the transform rotates) and yawed about world z.
    const M3D r_full = loop_transform.block<3, 3>(0, 0);
    const V3D t_full = loop_transform.block<3, 1>(0, 3);
    const V3D src_t = m_key_poses[cur_idx].t_global;
    const V3D displaced = r_full * src_t + t_full;
    const M3D r_yaw = yawRotation(yawOf(r_full));
    const V3D target_pos(displaced.x(), displaced.y(), src_t.z());
    const V3D t_planar = target_pos - r_yaw * src_t;
    loop_transform = toMatrix(r_yaw, t_planar);
  }

  LoopPair one_pair;
  one_pair.source_id = cur_idx;
  one_pair.target_id = loop_idx;
  one_pair.score = score;
  one_pair.gicp_r = loop_transform.block<3, 3>(0, 0);
  one_pair.gicp_t = loop_transform.block<3, 1>(0, 3);

  // Corrected source pose, then relative to the target (for the 6-DOF BetweenFactor).
  M3D r_refined = one_pair.gicp_r * m_key_poses[cur_idx].r_global;
  V3D t_refined = one_pair.gicp_r * m_key_poses[cur_idx].t_global + one_pair.gicp_t;

  one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
  one_pair.t_offset =
    m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);

  // World-frame (dx, dy, dyaw) of the corrected source vs the target, for PlanarLoopFactor.
  const V3D d = t_refined - m_key_poses[loop_idx].t_global;
  const double dyaw = yawOf(r_refined) - yawOf(m_key_poses[loop_idx].r_global);
  one_pair.planar_meas = V3D(d.x(), d.y(), PlanarLoopFactor::wrapAngle(dyaw));
  return one_pair;
}

gtsam::SharedNoiseModel PoseGraphOptimizer::robustify(const gtsam::SharedNoiseModel & model) const
{
  namespace me = gtsam::noiseModel::mEstimator;
  const double k = m_config.loop_robust_kernel_scale;
  if (m_config.loop_robust_kernel == "huber")
    return gtsam::noiseModel::Robust::Create(me::Huber::Create(k), model);
  if (m_config.loop_robust_kernel == "cauchy")
    return gtsam::noiseModel::Robust::Create(me::Cauchy::Create(k), model);
  return model;
}

void PoseGraphOptimizer::smoothAndUpdate()
{
  bool has_loop = !m_cache_pairs.empty();

  if (has_loop) {
    for (LoopPair & pair : m_cache_pairs) {
      // Pose3 tangent order (roll, pitch, yaw, x, y, z). The 6-DOF fallback keeps the fork's
      // "variance = fitness" on x / y / yaw but a tight value on roll / pitch / z, since the
      // chain is too compliant in z to take a tilt or z request from a loop.
      if (m_config.loop_planar_correction) {
        // Fixed x / y and yaw sigmas: "variance = fitness" left the loop too weak to move either.
        const double xy_var = m_config.loop_noise_xy_sigma_m > 0.0
                                ? m_config.loop_noise_xy_sigma_m * m_config.loop_noise_xy_sigma_m
                                : pair.score;
        const double yaw_sigma = m_config.loop_noise_yaw_sigma_deg * M_PI / 180.0;
        const double yaw_var =
          m_config.loop_noise_yaw_sigma_deg > 0.0 ? yaw_sigma * yaw_sigma : pair.score;
        m_graph.add(PlanarLoopFactor(
          pair.target_id, pair.source_id, pair.planar_meas,
          robustify(
            gtsam::noiseModel::Diagonal::Variances(gtsam::Vector3(xy_var, xy_var, yaw_var)))));
        continue;
      }
      const double rpz = m_config.loop_noise_var_roll_pitch_z;
      gtsam::Vector6 var;
      var << rpz, rpz, pair.score, pair.score, pair.score, rpz;
      m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        pair.target_id, pair.source_id,
        gtsam::Pose3(gtsam::Rot3(pair.r_offset), gtsam::Point3(pair.t_offset)),
        robustify(gtsam::noiseModel::Diagonal::Variances(var))));
    }
    std::vector<LoopPair>().swap(m_cache_pairs);
  }

  // One iSAM2 update() is ONE Gauss-Newton-style step, not a solve: it adds the new factors /
  // values (first call only), relinearises variables that moved more than relinearizeThreshold
  // (checked every call, relinearizeSkip = 1) and re-solves the affected Bayes-tree part. Each
  // further empty update() steps again from the improved linearisation.
  //  - No loop: the new keyframe starts close, so two calls suffice (as in LIO-SAM).
  //  - Loop: the correction is spread along the chain and the robust kernel down-weights the
  //    first steps, so iterate until the newest keyframe settles (see kMinLoopIterations).
  // Each step after a loop relinearises most of the chain; the loop cool-down keeps that rare.
  m_isam2->update(m_graph, m_initial_values);
  m_isam2->update();
  m_last_loop_iterations = 0;
  if (has_loop) {
    const gtsam::Key newest = m_key_poses.size() - 1;
    gtsam::Pose3 prev = m_isam2->calculateEstimate<gtsam::Pose3>(newest);
    for (int i = 0; i < kMaxLoopIterations; i++) {
      m_isam2->update();
      m_last_loop_iterations = i + 1;
      const gtsam::Pose3 cur = m_isam2->calculateEstimate<gtsam::Pose3>(newest);
      const bool settled = prev.localCoordinates(cur).norm() < kLoopSettledTangent;
      prev = cur;
      if (settled && m_last_loop_iterations >= kMinLoopIterations) break;
    }
  }
  m_graph.resize(0);
  m_initial_values.clear();

  gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
  for (size_t i = 0; i < m_key_poses.size(); i++) {
    gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
    m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
    m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
  }

  const KeyPoseWithCloud & last_item = m_key_poses.back();
  m_r_offset = last_item.r_global * last_item.r_local.transpose();
  m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}
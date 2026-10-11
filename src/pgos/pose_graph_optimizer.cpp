#include "syncai_mapping/pgos/pose_graph_optimizer.h"

#include <Eigen/Eigenvalues>
#include <algorithm>

namespace
{
// Scan Context: candidates preselected by ring key before the full,
// rotation-searching comparison.
constexpr size_t kScRingKeyCandidates = 10;
// A Scan Context candidate's coarse stage, relative to the fine one.
constexpr double kScCoarseFactor = 3.0;
// smoothAndUpdate after a loop: at least kMinLoopIterations extra iSAM2
// steps (the count the upstream code always ran), then until the newest
// keyframe's estimate moves less than kLoopSettledTangent (tangent-space
// norm: rad and m) between steps, at most kMaxLoopIterations.
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

  // Same settings as syncai_localizer's refine stage, which is the other
  // small_gicp user in the workspace. setNumThreads() has to come before the
  // first setInputTarget() (the kd-tree builder reads it).
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
  m_gicp_coarse.setMaxCorrespondenceDistance(kScCoarseFactor * m_config.loop_gicp_max_corr_dist);
  m_gicp_coarse.setTransformationEpsilon(1e-6);
}

bool PoseGraphOptimizer::isKeyPose(const PoseWithTime & pose)
{
  // The first pose is accepted unconditionally
  if (m_key_poses.size() == 0) return true;

  const KeyPoseWithCloud & last_item = m_key_poses.back();

  // Compute the translation and rotation deltas
  double delta_trans = (pose.t - last_item.t_local).norm();
  double delta_deg =
    Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;

  // Either one exceeding its threshold makes this a key pose
  if (delta_trans > m_config.keyframe_delta_m || delta_deg > m_config.keyframe_delta_deg)
    return true;

  return false;
}

bool PoseGraphOptimizer::addKeyPose(const CloudWithPose & cloud_with_pose)
{
  bool is_key_pose = isKeyPose(cloud_with_pose.pose);
  if (!is_key_pose) return false;

  size_t idx = m_key_poses.size();  // Index of the new node

  /**
    * m_r_offset / m_t_offset is the local -> global offset, computed after the previous round of
    * optimisation. The offset has to be applied because every other node in the graph lives in
    * the global frame; if the new node used its local pose directly as the initial value it would
    * be inconsistent with the rest of the graph, so it is moved into the global frame with the
    * last offset first.
    */
  M3D init_r = m_r_offset * cloud_with_pose.pose.r;
  V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;

  // Add the initial value
  m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));

  // Add the factor
  if (idx == 0) {
    // First frame -> prior constraint (PriorFactor)
    // A PriorFactor is an "absolute constraint": it declares outright that node 0 belongs at
    // this absolute pose
    gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
    m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
  } else {
    // Every later frame -> odometry constraint (BetweenFactor)
    // A BetweenFactor does not say where idx is, only that the relative pose from idx-1 to idx
    // should be this one
    const KeyPoseWithCloud & last_item = m_key_poses.back();

    // Relative rotation
    M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
    // Relative translation
    V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);

    // i.e. "odom says the robot moved this much between these two keyframes".
    // Noise variances are ordered (roll, pitch, yaw, x, y, z)
    // = (1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6). As standard deviations: rotation
    // ~0.06 deg, x/y ~1 cm, z ~1 mm. In other words, the short-range relative
    // odom measurement is trusted very strongly, especially along z.
    gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
    m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
  }

  // Tilt anchor: world "up" in this keyframe's body frame, as the LIO
  // measured it. Taken from the LOCAL (LIO) rotation, not init_r: the local
  // frame is gravity-aligned by construction, while init_r carries whatever
  // the last optimisation did. The global frame's z is the local frame's z
  // (node 0's prior is its local pose and loops only rotate about z), so the
  // two "ups" are the same vector. See gravity_prior_factor.h.
  if (m_config.keyframe_tilt_sigma_deg > 0.0) {
    const double s = m_config.keyframe_tilt_sigma_deg * M_PI / 180.0;
    const V3D up = cloud_with_pose.pose.r.transpose() * V3D::UnitZ();
    m_graph.add(GravityPriorFactor(
      idx, gtsam::Vector3(up.x(), up.y(), up.z()), gtsam::noiseModel::Isotropic::Sigma(3, s)));
  }

  // Finally, store the keyframe
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
    // Level the scan with the LIO's own tilt (gravity-aligned by
    // construction), keeping its yaw: the descriptor then depends on the
    // place and the heading only, and the heading is what distance() finds.
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
  std::vector<LoopCandidate> ret;
  const KeyPoseWithCloud & cur = m_key_poses[cur_idx];
  pcl::PointXYZ cur_pt;
  cur_pt.x = cur.t_global(0);
  cur_pt.y = cur.t_global(1);
  cur_pt.z = cur.t_global(2);

  // Every older frame except the current one
  pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < cur_idx; i++) {
    pcl::PointXYZ pt;
    pt.x = m_key_poses[i].t_global(0);
    pt.y = m_key_poses[i].t_global(1);
    pt.z = m_key_poses[i].t_global(2);
    key_poses_cloud->push_back(pt);
  }

  // Radius search, nearest first
  // {TODO} Possible performance hotspot: every call rebuilds the whole KD-tree (all historical
  // keyframes are inserted again from scratch)
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(key_poses_cloud);
  std::vector<int> ids;
  std::vector<float> sqdists;
  if (kdtree.radiusSearch(cur_pt, m_config.loop_search_radius, ids, sqdists) == 0) return ret;

  // Close in space and more than loop_time_tresh apart in time. Neighbours of
  // an already chosen candidate would be verified against nearly the same
  // submap, so candidates must be more than a submap apart in index: then
  // the extra ones are other passes through this place.
  const int min_gap = 2 * m_config.loop_submap_half_range + 1;
  const size_t max_candidates = static_cast<size_t>(std::max(1, m_config.loop_max_candidates));
  for (int idx : ids) {
    if (std::abs(cur.time - m_key_poses[idx].time) <= m_config.loop_time_tresh) continue;
    bool near_chosen = false;
    for (const LoopCandidate & c : ret)
      if (std::abs(idx - static_cast<int>(c.target_id)) < min_gap) near_chosen = true;
    if (near_chosen) continue;
    ret.push_back({static_cast<size_t>(idx), cur.r_global, cur.t_global, false, 0.0});
    if (ret.size() >= max_candidates) break;
  }
  return ret;
}

bool PoseGraphOptimizer::scanContextCandidate(size_t cur_idx, LoopCandidate & candidate)
{
  const KeyPoseWithCloud & cur = m_key_poses[cur_idx];
  if (cur.sc_desc.size() == 0) return false;

  // Ring keys are rotation-invariant: a cheap preselection. With the drift
  // gate on, only keyframes it could pass are considered: the guess puts the
  // current keyframe on the target, so the correction is about their
  // distance, give or take the coarse stage's reach. That keeps symmetric
  // places far away from ever costing a registration.
  const double reach = kScCoarseFactor * m_config.loop_gicp_max_corr_dist;
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

  // Initial guess: the current keyframe at the target's position, with the
  // target's heading turned by the descriptor's yaw and its own tilt kept.
  // The position error (a revisit is rarely the exact spot) is the coarse
  // GICP stage's job.
  const KeyPoseWithCloud & tgt = m_key_poses[best_idx];
  const M3D cur_tilt = yawRotation(-yawOf(cur.r_global)) * cur.r_global;
  candidate.target_id = best_idx;
  candidate.r_init =
    yawRotation(yawOf(tgt.r_global) + ScanContext::yawOfShift(best_shift)) * cur_tilt;
  candidate.t_init = tgt.t_global;
  candidate.scan_context = true;
  candidate.sc_dist = best_dist;
  return true;
}

bool PoseGraphOptimizer::verifyCandidate(
  size_t cur_idx, const LoopCandidate & candidate, LoopCandidateReport & report, M4D & correction)
{
  report.target_id = candidate.target_id;
  report.source_id = cur_idx;
  report.scan_context = candidate.scan_context;
  report.sc_dist = candidate.sc_dist;
  const KeyPoseWithCloud & cur = m_key_poses[cur_idx];

  // Target: the 2 * half_range + 1 keyframes around the old one, each in its
  // own world pose, stacked and voxelised. Source: the current scan at the
  // candidate's initial pose.
  CloudType::Ptr target = getSubMap(
    candidate.target_id, m_config.loop_submap_half_range, m_config.loop_submap_resolution);
  CloudType::Ptr source(new CloudType);
  pcl::transformPointCloud(
    *cur.body_cloud, *source, candidate.t_init, Eigen::Quaterniond(candidate.r_init));
  if (m_config.loop_submap_resolution > 0)
    source = voxelized(source, m_config.loop_submap_resolution);

  M4F guess = M4F::Identity();
  CloudType::Ptr aligned(new CloudType);
  if (candidate.scan_context) {
    // Coarse first, as Cartographer matches against its low-resolution grid
    // before the high-resolution one: the guess can be metres off, beyond the
    // fine stage's correspondence distance.
    const double coarse_res = kScCoarseFactor * std::max(m_config.loop_submap_resolution, 0.05);
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

  // Hessian order (rx, ry, rz, tx, ty, tz), perturbing the source, which is
  // already in world coordinates: the (tx, ty) block is world x / y.
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

  // Registered pose = transform * initial pose; the correction maps the
  // keyframe's graph pose onto it (identity initial pose -> transform).
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

  // The trajectory is still too short to have possibly looped back on itself.
  if (m_key_poses.size() < 10) return;

  // cool down time
  if (m_config.loop_search_period > 0.0) {
    if (m_history_pairs.size() > 0) {
      double current_time = m_key_poses.back().time;
      double last_time = m_key_poses[m_history_pairs.back().second].time;
      if (current_time - last_time < m_config.loop_search_period) return;
    }
  }

  // "Close in space, far apart in time" alone cannot be trusted (two
  // corridors can be close in position too): every candidate is verified by
  // registration, and among those that pass, the best inlier ratio wins.
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

  // Cache it; smoothAndUpdate adds it to the graph
  m_cache_pairs.push_back(makeLoopPair(cur_idx, best_target, best_correction, best_fitness));

  // Used by the markers and the cool-down
  m_history_pairs.emplace_back(best_target, cur_idx);
}

LoopPair PoseGraphOptimizer::makeLoopPair(
  size_t cur_idx, size_t loop_idx, const M4D & correction, double score)
{
  // The correction transform computed by the registration, expressed in the
  // global frame: it maps the source keyframe's world cloud onto the submap.
  M4D loop_transform = correction;
  if (m_config.loop_planar_correction) {
    // Keep only what the loop is trusted for: where the source keyframe should
    // sit in x / y and how it should be yawed. Its world z, roll and pitch
    // stay exactly as the graph has them now. Built so that the source POSE
    // is displaced by the registration's own xy displacement (not by the raw
    // translation column, which is meaningless 20 m from the origin when the
    // transform carries a rotation) and yawed about the world z axis.
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

  // First apply the correction to the current frame's global pose, giving the corrected current
  // pose
  M3D r_refined = one_pair.gicp_r * m_key_poses[cur_idx].r_global;
  V3D t_refined = one_pair.gicp_r * m_key_poses[cur_idx].t_global + one_pair.gicp_t;

  // Then express it as the relative pose of source with respect to target
  one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
  one_pair.t_offset =
    m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);

  // And in the world frame, for the planar factor: where the corrected source
  // sits relative to the target in x / y, and by how much it is yawed.
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

  // Add the loop-closure factors
  if (has_loop) {
    for (LoopPair & pair : m_cache_pairs) {
      // Pose3 tangent order is (roll, pitch, yaw, x, y, z). The fork used the
      // fitness score as the variance on all six; roll / pitch / z now have
      // their own, tight, value -- a loop is trusted in the plane, and the
      // chain is far too compliant in z (see loop_planar_correction) to be
      // handed a z or tilt request with sigma ~0.3 m / 18 deg.
      if (m_config.loop_planar_correction) {
        // World-frame x / y / yaw only; z / roll / pitch are not in the error.
        // x / y and yaw each have their own sigma (loop_noise_xy_sigma_m,
        // loop_noise_yaw_sigma_deg); the fork's "variance = fitness" made
        // the loop too weak to move either -- see the config comments.
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
    // Clear cache_pairs
    std::vector<LoopPair>().swap(m_cache_pairs);
  }

  // smooth and mapping
  // iSAM2 (Incremental Smoothing and Mapping) is incremental: internally it uses a Bayes tree
  // and only relinearises and re-solves the part that changed.
  //
  // Why update() is called more than once: one call is ONE Gauss-Newton-style
  // step, not a solve to convergence. It adds the new factors / values (only
  // the first call passes any), relinearises the variables that drifted more
  // than relinearizeThreshold (0.01) from their linearisation point
  // (relinearizeSkip = 1: checked on every call), and re-solves the affected
  // part of the Bayes tree for a new delta. The graph is nonlinear (the
  // rotations), so a step taken from a linearisation point far from the
  // solution is only approximate; each further empty update() relinearises at
  // the improved estimate and steps again.
  //  - No loop: the new keyframe's initial value comes from the last offset
  //    and its factors are odometry + tilt anchor, so it starts close and one
  //    step nearly converges; the second call is the usual extra iteration
  //    (the same pattern as LIO-SAM).
  //  - Loop: the loop factor can ask to move the keyframe tens of cm / a few
  //    degrees, spread along the whole chain, so the first step is coarse,
  //    and with a robust kernel the first steps are also down-weighted (the
  //    kernel sees the whole correction as residual until it is applied).
  //    So it iterates until the newest keyframe settles: at least the four
  //    extra steps upstream always ran, at most kMaxLoopIterations.
  // Each iteration costs a relinearisation of the affected variables (after
  // a loop, most of the chain); the loop cool-down keeps that infrequent.
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

  // Write the optimised result back to the keyframes
  gtsam::Values estimate_values = m_isam2->calculateBestEstimate();  // Best estimate
  for (size_t i = 0; i < m_key_poses.size(); i++) {
    gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
    m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();  // Overwrite global
    m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
  }

  // Recompute the offset and store the updated value
  const KeyPoseWithCloud & last_item = m_key_poses.back();
  m_r_offset = last_item.r_global * last_item.r_local.transpose();
  m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}
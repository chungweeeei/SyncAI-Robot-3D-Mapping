#include "syncai_mapping/pgos/session_aligner.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <unordered_set>

namespace
{
// The coarse score: a scan point counts when it lands in an occupied cell of the saved map or in
// one beside it, so a candidate up to about a cell off its true place still scores. Half a
// candidate_spacing would do for a robot on the old track; it rarely is.
constexpr double kScoreVoxel = 0.5;
// Scan points scored per candidate.
constexpr size_t kScorePoints = 400;
// Two results further apart than this are different places (the ambiguity test), and a chosen
// candidate suppresses its neighbours within twice the candidate grid.
constexpr double kDistinctDist = 1.0;
constexpr double kDistinctYaw = 10.0 * M_PI / 180.0;
// The coarse map is re-voxelised whenever this many points have been stacked, as the live merge
// does, so it stays the size of the place.
constexpr size_t kCoarseMapVoxelEvery = 2'000'000;

double wrapAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// 21 bits a coordinate: +-500 km at kScoreVoxel.
uint64_t cellKey(int64_t x, int64_t y, int64_t z)
{
  constexpr int64_t kBias = 1 << 20;
  constexpr uint64_t kMask = (1ull << 21) - 1;
  return ((static_cast<uint64_t>(x + kBias) & kMask) << 42) |
         ((static_cast<uint64_t>(y + kBias) & kMask) << 21) |
         (static_cast<uint64_t>(z + kBias) & kMask);
}

int64_t cellOf(double v)
{
  return static_cast<int64_t>(std::floor(v / kScoreVoxel));
}

struct Candidate
{
  V3D position;       // the body in the map
  double yaw_offset;  // of map <- odom
  double score = 0.0;
};

// The saved keyframes within `radius` (xy) and `z_band` of `centre`, in the map frame.
CloudType::Ptr targetAround(
  const std::vector<KeyPoseWithCloud> & prior, const V3D & centre, double radius, double z_band,
  double resolution)
{
  CloudType::Ptr ret(new CloudType);
  CloudType world;
  for (const KeyPoseWithCloud & kp : prior) {
    if ((kp.t_global - centre).head<2>().norm() > radius) continue;
    if (std::abs(kp.t_global.z() - centre.z()) > z_band) continue;
    pcl::transformPointCloud(*kp.body_cloud, world, kp.t_global, Eigen::Quaterniond(kp.r_global));
    *ret += world;
  }
  if (resolution > 0 && !ret->empty()) ret = voxelized(ret, resolution);
  return ret;
}
}  // namespace

SessionAligner::SessionAligner(const SessionAlignerConfig & config) : m_config(config)
{
  // Whatever the caller copied into the gates: here a registration is judged by them alone.
  m_config.gates.require_convergence = false;
  configureGicp(
    m_gicp, m_config.gicp_num_threads, m_config.gicp_num_neighbors, m_config.gates.max_corr_dist);
  configureGicp(
    m_gicp_coarse, m_config.gicp_num_threads, m_config.gicp_num_neighbors,
    kCoarseFactor * m_config.gates.max_corr_dist);
}

SessionAlignResult SessionAligner::align(
  const std::vector<KeyPoseWithCloud> & prior, const SessionAlignRequest & request)
{
  using Clock = std::chrono::steady_clock;
  SessionAlignResult result;
  result.used_hint = request.use_hint;
  if (prior.empty() || !request.cloud_odom || request.cloud_odom->empty()) {
    result.message = "Nothing to align: no saved keyframes or no scan.";
    return result;
  }
  const double body_yaw = yawOf(request.r_body);
  // A measured hint brings its own, tighter, window (SessionAlignRequest::hint_radius).
  const bool measured_hint = request.use_hint && request.hint_radius > 0.0;
  const double hint_radius = measured_hint ? request.hint_radius : m_config.hint_radius;
  const double hint_yaw_window =
    (measured_hint ? request.hint_yaw_deg : m_config.hint_yaw_deg) * M_PI / 180.0;

  // ---- Candidate positions ----
  //
  // Saved keyframe positions, thinned greedily in file order (as GlobalLocalizer does): where the
  // robot once was is where it can be, at the height it had there.
  auto near_hint = [&](const V3D & p, double radius) {
    return (p - request.hint_position).head<2>().norm() <= radius &&
           std::abs(p.z() - request.hint_position.z()) <= m_config.target_z_band;
  };
  std::vector<V3D> positions;
  if (request.use_hint) {
    // The hint itself first, at the height of the nearest saved keyframe on its storey.
    double best = m_config.target_radius;
    bool found = false;
    V3D at_hint = request.hint_position;
    for (const KeyPoseWithCloud & kp : prior) {
      if (!near_hint(kp.t_global, m_config.target_radius)) continue;
      const double d = (kp.t_global - request.hint_position).head<2>().norm();
      if (d <= best) {
        best = d;
        at_hint.z() = kp.t_global.z();
        found = true;
      }
    }
    if (!found) {
      char buf[160];
      std::snprintf(
        buf, sizeof(buf),
        "The given position (%.1f, %.1f) is more than %.0f m from anything in the saved map.",
        request.hint_position.x(), request.hint_position.y(), m_config.target_radius);
      result.message = buf;
      return result;
    }
    positions.push_back(at_hint);
  }
  for (const KeyPoseWithCloud & kp : prior) {
    if (request.use_hint && !near_hint(kp.t_global, hint_radius)) continue;
    bool kept = true;
    for (const V3D & p : positions)
      if ((p - kp.t_global).norm() < m_config.candidate_spacing) {
        kept = false;
        break;
      }
    if (kept) positions.push_back(kp.t_global);
  }

  // ---- Candidate yaws: of map <- odom, so the body's heading in the map less its odom one ----
  std::vector<double> yaw_offsets;
  const double step = std::max(m_config.yaw_step_deg, 1.0) * M_PI / 180.0;
  if (request.use_hint) {
    const double centre = request.hint_yaw - body_yaw;
    const int n = static_cast<int>(std::floor(hint_yaw_window / step + 1e-9));
    for (int k = -n; k <= n; k++) yaw_offsets.push_back(wrapAngle(centre + k * step));
  } else {
    const int n = std::max(1, static_cast<int>(std::round(2.0 * M_PI / step)));
    for (int k = 0; k < n; k++) yaw_offsets.push_back(wrapAngle(k * 2.0 * M_PI / n));
  }

  // ---- The coarse score ----
  //
  // Occupied cells of the saved map near any candidate, dilated by one cell. A hash set, not a
  // KD-tree: positions x yaws x points is tens of millions of lookups on a large site.
  std::unordered_set<uint64_t> occupied;
  {
    const double reach = request.use_hint ? hint_radius + m_config.target_radius : -1.0;
    CloudType::Ptr coarse_map(new CloudType);
    CloudType world;
    size_t since_voxel = 0;
    for (const KeyPoseWithCloud & kp : prior) {
      if (reach > 0.0 && !near_hint(kp.t_global, reach)) continue;
      const CloudType::Ptr & src = kp.preview_cloud ? kp.preview_cloud : kp.body_cloud;
      pcl::transformPointCloud(*src, world, kp.t_global, Eigen::Quaterniond(kp.r_global));
      *coarse_map += world;
      since_voxel += world.size();
      if (since_voxel >= kCoarseMapVoxelEvery) {
        coarse_map = voxelized(coarse_map, kScoreVoxel);
        since_voxel = 0;
      }
    }
    occupied.reserve(coarse_map->size() * 8);
    for (const PointType & p : coarse_map->points) {
      const int64_t cx = cellOf(p.x), cy = cellOf(p.y), cz = cellOf(p.z);
      for (int64_t dx = -1; dx <= 1; dx++)
        for (int64_t dy = -1; dy <= 1; dy++)
          for (int64_t dz = -1; dz <= 1; dz++) occupied.insert(cellKey(cx + dx, cy + dy, cz + dz));
    }
  }

  // The scan about the body, thinned to a few hundred points spread over what it sees.
  std::vector<V3D> score_points;
  {
    const CloudType::Ptr thin = voxelized(request.cloud_odom, kScoreVoxel);
    const size_t stride = std::max<size_t>(1, thin->size() / kScorePoints);
    for (size_t i = 0; i < thin->size(); i += stride) {
      const PointType & p = thin->points[i];
      score_points.push_back(V3D(p.x, p.y, p.z) - request.t_body);
    }
  }

  std::vector<Candidate> candidates(positions.size() * yaw_offsets.size());
  const int n_positions = static_cast<int>(positions.size());
  const int n_yaws = static_cast<int>(yaw_offsets.size());
#pragma omp parallel for schedule(dynamic, 8)
  for (int pi = 0; pi < n_positions; pi++) {
    for (int yi = 0; yi < n_yaws; yi++) {
      const double c = std::cos(yaw_offsets[yi]), s = std::sin(yaw_offsets[yi]);
      const V3D & pos = positions[pi];
      size_t hits = 0;
      for (const V3D & p : score_points) {
        const double x = c * p.x() - s * p.y() + pos.x();
        const double y = s * p.x() + c * p.y() + pos.y();
        if (occupied.count(cellKey(cellOf(x), cellOf(y), cellOf(p.z() + pos.z())))) hits++;
      }
      Candidate & cand = candidates[static_cast<size_t>(pi) * n_yaws + yi];
      cand.position = pos;
      cand.yaw_offset = yaw_offsets[yi];
      cand.score = score_points.empty() ? 0.0 : static_cast<double>(hits) / score_points.size();
    }
  }
  result.candidates = candidates.size();

  // Best first, a chosen one suppressing its neighbours: adjacent cells and yaws of one place
  // would otherwise take every registration.
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) { return a.score > b.score; });
  std::vector<Candidate> chosen;
  const size_t max_registrations = static_cast<size_t>(std::max(1, m_config.max_registrations));
  for (const Candidate & cand : candidates) {
    bool suppressed = false;
    for (const Candidate & c : chosen)
      if (
        (c.position - cand.position).norm() < 2.0 * m_config.candidate_spacing &&
        std::abs(wrapAngle(c.yaw_offset - cand.yaw_offset)) < 2.0 * step) {
        suppressed = true;
        break;
      }
    if (suppressed) continue;
    chosen.push_back(cand);
    if (chosen.size() >= max_registrations) break;
  }

  // ---- Registration ----
  struct Passed
  {
    size_t attempt;
    M3D r_offset;
    V3D t_offset;
  };
  std::vector<Passed> passed;
  const Clock::time_point deadline =
    Clock::now() + std::chrono::duration_cast<Clock::duration>(
                     std::chrono::duration<double>(std::max(m_config.timeout_sec, 0.0)));
  bool timed_out = false;
  // Candidates at one position (a hint's yaws) share their target.
  V3D target_centre = V3D::Constant(1e18);
  CloudType::Ptr target;
  for (const Candidate & cand : chosen) {
    if (!result.attempts.empty() && Clock::now() > deadline) {
      timed_out = true;
      break;
    }
    SessionAlignAttempt attempt;
    attempt.position = cand.position;
    attempt.yaw = wrapAngle(cand.yaw_offset + body_yaw);
    attempt.coarse_score = cand.score;

    if ((cand.position - target_centre).norm() > 1e-6) {
      target = targetAround(
        prior, cand.position, m_config.target_radius, m_config.target_z_band,
        m_config.gates.resolution);
      target_centre = cand.position;
    }
    // The scan at the candidate: odom rotated about z, the body on the candidate's position.
    const M3D r_init = yawRotation(cand.yaw_offset);
    const V3D t_init = cand.position - r_init * request.t_body;
    CloudType::Ptr source(new CloudType);
    pcl::transformPointCloud(*request.cloud_odom, *source, t_init, Eigen::Quaterniond(r_init));
    if (m_config.gates.resolution > 0) source = voxelized(source, m_config.gates.resolution);

    M4D transform = M4D::Identity();
    bool ok = false;
    if (target->empty()) {
      attempt.reject_reason = "no saved keyframe near";
    } else {
      ok = registerAndScore(
        m_gicp, m_gicp_coarse, source, target, true, m_config.gates, attempt.registration,
        transform);
      attempt.reject_reason = attempt.registration.reject_reason;
    }

    Passed p{result.attempts.size(), M3D::Identity(), V3D::Zero()};
    if (attempt.registration.registered) {
      // map <- odom as registered, all of it. What is not a rotation about z is the two runs'
      // ups disagreeing, and it is kept: the saved map's up is the one the run must take.
      const M4D full = transform * toMatrix(r_init, t_init);
      p.r_offset = full.block<3, 3>(0, 0);
      p.t_offset = full.block<3, 1>(0, 3);
      attempt.position = p.r_offset * request.t_body + p.t_offset;
      attempt.yaw = yawOf(p.r_offset * request.r_body);
      const M3D tilt = yawRotation(-yawOf(p.r_offset)) * p.r_offset;
      attempt.tilt_residual_deg = Eigen::AngleAxisd(tilt).angle() * 180.0 / M_PI;
    }
    if (ok && attempt.registration.source_inlier_ratio < m_config.min_source_inlier_ratio) {
      ok = false;
      attempt.reject_reason = "too little of the scan on the saved map";
    }
    if (ok && attempt.tilt_residual_deg > m_config.max_tilt_deg) {
      ok = false;
      attempt.reject_reason = "tilt (the two runs' up directions are too far apart)";
    }
    if (ok && request.use_hint) {
      // Within reach of what was asked: a registration that walked off to another place is not
      // an answer to this hint. An estimate gets the coarse stage's reach on top of its window,
      // a measurement nothing.
      const double reach =
        measured_hint ? hint_radius : hint_radius + kCoarseFactor * m_config.gates.max_corr_dist;
      const double yaw_reach = measured_hint ? hint_yaw_window : hint_yaw_window + step;
      if (
        (attempt.position - request.hint_position).head<2>().norm() > reach ||
        std::abs(wrapAngle(attempt.yaw - request.hint_yaw)) > yaw_reach) {
        ok = false;
        attempt.reject_reason = "registered away from the given position";
      }
    }
    attempt.passed = ok;
    result.attempts.push_back(attempt);
    if (ok) passed.push_back(p);
  }

  // ---- The verdict ----
  if (passed.empty()) {
    const SessionAlignAttempt * closest = nullptr;
    for (const SessionAlignAttempt & a : result.attempts)
      if (
        !closest || a.registration.source_inlier_ratio > closest->registration.source_inlier_ratio)
        closest = &a;
    result.message = request.use_hint
                       ? "Could not find the robot near the given position in the saved map"
                       : "Could not find the robot in the saved map";
    if (closest)
      result.message +=
        std::string(" (closest candidate rejected: ") + closest->reject_reason + ")";
    result.message += ".";
    if (timed_out) result.message += " The search ran out of time.";
    if (!request.use_hint)
      result.message += " Move it to a place the saved map covers, or give its position.";
    return result;
  }

  auto score = [&](const Passed & p) {
    return result.attempts[p.attempt].registration.source_inlier_ratio;
  };
  const Passed * best = &passed.front();
  for (const Passed & p : passed)
    if (score(p) > score(*best)) best = &p;
  const SessionAlignAttempt & best_attempt = result.attempts[best->attempt];

  if (!request.use_hint) {
    for (const Passed & p : passed) {
      const SessionAlignAttempt & other = result.attempts[p.attempt];
      const bool distinct = (other.position - best_attempt.position).norm() > kDistinctDist ||
                            std::abs(wrapAngle(other.yaw - best_attempt.yaw)) > kDistinctYaw;
      if (&p == best || !distinct || score(p) < m_config.ambiguity_ratio * score(*best)) continue;
      char buf[256];
      std::snprintf(
        buf, sizeof(buf),
        "This place looks like another one in the saved map: (%.1f, %.1f) and (%.1f, %.1f) fit "
        "about equally. Give the robot's position, or start from a more distinctive place.",
        best_attempt.position.x(), best_attempt.position.y(), other.position.x(),
        other.position.y());
      result.message = buf;
      return result;
    }
  }

  result.success = true;
  result.r_offset = best->r_offset;
  result.t_offset = best->t_offset;
  result.position = best_attempt.position;
  result.yaw = best_attempt.yaw;
  result.registration = best_attempt.registration;
  result.tilt_residual_deg = best_attempt.tilt_residual_deg;
  return result;
}

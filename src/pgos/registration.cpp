#include "syncai_mapping/pgos/registration.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <vector>

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

void configureGicp(
  GicpRegistration & gicp, int num_threads, int num_neighbors, double max_corr_dist)
{
  gicp.setNumThreads(num_threads);
  gicp.setCorrespondenceRandomness(num_neighbors);
  gicp.setRegistrationType("GICP");
  gicp.setMaximumIterations(50);
  gicp.setMaxCorrespondenceDistance(max_corr_dist);
  gicp.setTransformationEpsilon(1e-6);
}

bool registerAndScore(
  GicpRegistration & fine, GicpRegistration & coarse, const CloudType::Ptr & source,
  const CloudType::Ptr & target, bool coarse_first, const RegistrationGates & gates,
  RegistrationReport & report, M4D & transform)
{
  M4F guess = M4F::Identity();
  CloudType::Ptr aligned(new CloudType);
  if (coarse_first) {
    // Coarse first: the guess can be metres off, beyond the fine stage's correspondence distance.
    const double coarse_res = kCoarseFactor * std::max(gates.resolution, 0.05);
    coarse.setInputSource(voxelized(source, coarse_res));
    coarse.setInputTarget(voxelized(target, coarse_res));
    coarse.align(*aligned);
    if (!coarse.hasConverged() && gates.require_convergence) {
      report.reject_reason = "coarse GICP did not converge";
      return false;
    }
    guess = coarse.getFinalTransformation();
  }

  fine.setInputSource(source);
  fine.setInputTarget(target);
  fine.align(*aligned, guess);
  report.converged = fine.hasConverged();
  if (!report.converged && gates.require_convergence) {
    report.reject_reason = "GICP did not converge";
    return false;
  }
  report.registered = true;
  transform = fine.getFinalTransformation().cast<double>();

  // Overlap / inlier / fitness over the aligned source (see PoseGraphOptimizerConfig).
  pcl::KdTreeFLANN<PointType> tree;
  tree.setInputCloud(target);
  const double overlap_sq = gates.max_corr_dist * gates.max_corr_dist;
  const double inlier_sq = gates.inlier_dist * gates.inlier_dist;
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
  report.source_inlier_ratio =
    aligned->empty() ? 0.0 : static_cast<double>(n_inlier) / aligned->size();
  report.fitness = n_overlap == 0 ? 1e9 : sum_sq / n_overlap;

  // Hessian order (rx, ry, rz, tx, ty, tz), perturbing the already world-frame source: the
  // (tx, ty) block is world x / y.
  const Eigen::Matrix2d h_xy = fine.getFinalHessian().block<2, 2>(3, 3);
  const Eigen::Vector2d ev = Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(h_xy).eigenvalues();
  report.xy_info_ratio = ev(1) > 0.0 ? std::max(ev(0), 0.0) / ev(1) : 0.0;

  if (report.overlap_ratio < gates.min_overlap_ratio) {
    report.reject_reason = "overlap ratio";
    return false;
  }
  if (report.inlier_ratio < gates.min_inlier_ratio) {
    report.reject_reason = "inlier ratio";
    return false;
  }
  if (report.fitness > gates.score_tresh) {
    report.reject_reason = "fitness";
    return false;
  }
  if (gates.min_xy_info_ratio > 0.0 && report.xy_info_ratio < gates.min_xy_info_ratio) {
    report.reject_reason = "degenerate (xy info ratio)";
    return false;
  }
  return true;
}

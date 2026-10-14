#pragma once
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <functional>

/**
 * Unary factor pinning one keyframe's tilt (roll / pitch w.r.t. gravity) to the LIO estimate;
 * yaw and position stay free.
 *
 * The odometry BetweenFactors live in the body frame (pitched ~15 deg) and PlanarLoopFactor
 * constrains only world x / y / yaw, so a loop's yaw correction can otherwise be realised with
 * a rotation carrying a little pitch, and the chain then climbs or sinks as it drives. Point-LIO
 * observes tilt from the IMU at every instant, so it is an absolute measurement; with every
 * keyframe anchored, heading can only change by rotating about world z.
 *
 * Error (3-dim, rank 2): R^T e_z - g, with g = R_lio^T e_z (world up in the body frame at
 * insertion). A 3-vector avoids Euler singularities; its component along g is second order.
 * Numerical Jacobian, as in PlanarLoopFactor.
 */
class GravityPriorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  GravityPriorFactor(
    gtsam::Key key, const gtsam::Vector3 & up_in_body, const gtsam::SharedNoiseModel & model)
  : Base(model, key), m_up(up_in_body)
  {
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose, boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    const gtsam::Vector3 up = m_up;
    const std::function<gtsam::Vector3(const gtsam::Pose3 &)> err = [up](const gtsam::Pose3 & p) {
      return gtsam::Vector3(p.rotation().matrix().transpose() * gtsam::Vector3::UnitZ() - up);
    };
    if (H) *H = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(err, pose);
    return err(pose);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GravityPriorFactor(*this)));
  }

private:
  gtsam::Vector3 m_up;
};

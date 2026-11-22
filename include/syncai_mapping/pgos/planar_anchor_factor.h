#pragma once
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <functional>

#include "syncai_mapping/pgos/planar_loop_factor.h"

/**
 * The unary twin of PlanarLoopFactor: one keyframe's world x / y and yaw held to a fixed
 * measurement, for a resumed run (PoseGraphOptimizer::loadPrior), where the other end of the
 * constraint is not a variable:
 *  - a loop against a keyframe of the saved map, whose pose is a constant. The measurement is
 *    that pose plus the loop's planar offset, so the error is PlanarLoopFactor's with the target
 *    fixed;
 *  - the run's first keyframe, held where the session alignment put it.
 *
 * Not a PriorFactor<Pose3> with tight roll / pitch / z noise, for PlanarLoopFactor's reason: its
 * error lives in the body frame, pitched ~15 deg here, so an x correction leaks into world z.
 *
 * Error (3-dim): [ t.xy - m.xy ; wrap(yaw - m.yaw) ], yaw = atan2(R10, R00). Numerical Jacobian,
 * as in PlanarLoopFactor.
 */
class PlanarAnchorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  // meas = (x, y, yaw) in the world frame.
  PlanarAnchorFactor(
    gtsam::Key key, const gtsam::Vector3 & meas, const gtsam::SharedNoiseModel & model)
  : Base(model, key), m_meas(meas)
  {
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose, boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    const gtsam::Vector3 meas = m_meas;
    const std::function<gtsam::Vector3(const gtsam::Pose3 &)> err = [meas](const gtsam::Pose3 & p) {
      gtsam::Vector3 e;
      e.head<2>() = p.translation().head<2>() - meas.head<2>();
      e(2) = PlanarLoopFactor::wrapAngle(PlanarLoopFactor::yawOf(p) - meas(2));
      return e;
    };
    if (H) *H = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(err, pose);
    return err(pose);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new PlanarAnchorFactor(*this)));
  }

  const gtsam::Vector3 & measured() const { return m_meas; }

private:
  gtsam::Vector3 m_meas;
};

/**
 * One keyframe's world z held to a measurement; x / y and the rotation stay free.
 *
 * In a run begun empty nothing needs it: node 0's prior fixes z and no loop touches it. A resumed
 * run has no such prior (its first keyframe is anchored by PlanarAnchorFactor, which says nothing
 * about z), so this is what fixes the run's height: on that first keyframe always, and on the
 * source of a loop against the saved map with resume_loop_z_sigma_m.
 *
 * A world-frame scalar, so it has neither failure the 6-DOF loop edge had: it asks for no tilt,
 * and nothing leaks into it through the lidar's pitch.
 *
 * Error (1-dim): t.z - z. Numerical Jacobian, as in PlanarLoopFactor.
 */
class HeightPriorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  HeightPriorFactor(gtsam::Key key, double z, const gtsam::SharedNoiseModel & model)
  : Base(model, key), m_z(z)
  {
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose, boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    const double z = m_z;
    const std::function<gtsam::Vector1(const gtsam::Pose3 &)> err = [z](const gtsam::Pose3 & p) {
      return gtsam::Vector1(p.translation().z() - z);
    };
    if (H) *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(err, pose);
    return err(pose);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new HeightPriorFactor(*this)));
  }

  double measured() const { return m_z; }

private:
  double m_z;
};

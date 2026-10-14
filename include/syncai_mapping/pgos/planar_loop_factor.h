#pragma once
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <functional>

/**
 * Loop-closure factor on what a loop is trusted for only: the world-frame x / y offset and the
 * yaw difference between target (old) and source (new) keyframes. z / roll / pitch are not in the
 * error, so the loop cannot bend the chain in pitch.
 *
 * Not a BetweenFactor<Pose3> with tight roll / pitch / z noise: its error lives in the body frame,
 * pitched ~15 deg here, so x corrections leak ~sin(15 deg) = 0.26 of themselves into world z.
 *
 * Error (3-dim): [ (t_s - t_t).xy - m.xy ; wrap(yaw(s) - yaw(t) - m.yaw) ], yaw = atan2(R10, R00).
 * Numerical Jacobians: few loops per run, and no analytic expression to get wrong.
 */
class PlanarLoopFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

  // meas = (dx, dy, dyaw): source relative to target, in the world frame.
  PlanarLoopFactor(
    gtsam::Key target, gtsam::Key source, const gtsam::Vector3 & meas,
    const gtsam::SharedNoiseModel & model)
  : Base(model, target, source), m_meas(meas)
  {
  }

  static double yawOf(const gtsam::Pose3 & p)
  {
    const gtsam::Matrix3 R = p.rotation().matrix();
    return std::atan2(R(1, 0), R(0, 0));
  }

  static double wrapAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // What the two poses currently say, in the measurement's parametrisation.
  static gtsam::Vector3 planarDelta(const gtsam::Pose3 & target, const gtsam::Pose3 & source)
  {
    gtsam::Vector3 d;
    d.head<2>() = (source.translation() - target.translation()).head<2>();
    d(2) = wrapAngle(yawOf(source) - yawOf(target));
    return d;
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & target, const gtsam::Pose3 & source,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override
  {
    const gtsam::Vector3 meas = m_meas;
    const std::function<gtsam::Vector3(const gtsam::Pose3 &, const gtsam::Pose3 &)> err =
      [meas](const gtsam::Pose3 & t, const gtsam::Pose3 & s) {
        gtsam::Vector3 e = planarDelta(t, s) - meas;
        e(2) = wrapAngle(e(2));
        return e;
      };
    if (H1)
      *H1 = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
        err, target, source);
    if (H2)
      *H2 = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
        err, target, source);
    return err(target, source);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new PlanarLoopFactor(*this)));
  }

  const gtsam::Vector3 & measured() const { return m_meas; }

private:
  gtsam::Vector3 m_meas;
};

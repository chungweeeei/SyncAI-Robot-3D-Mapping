// SessionAligner on the synthetic building (synthetic_hall.h): the saved map is a lap of the
// rectangle at its ground-truth poses, and the robot stands somewhere in the hall with an
// odometry frame of its own. The aligner must return the map <- odom transform, or refuse.
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "syncai_mapping/pgos/session_aligner.h"
#include "synthetic_hall.h"

namespace
{
using namespace synthetic_hall;

const World & world()
{
  static const World w;
  return w;
}

// Without its furniture the hall looks the same from a place and from its mirror image.
const World & symmetricWorld()
{
  static const World w(false);
  return w;
}

// Every other pose of one lap: a keyframe each 1.1 m.
std::vector<KeyPoseWithCloud> savedMap(const World & w)
{
  std::mt19937 rng(21);
  std::vector<KeyPoseWithCloud> ret;
  const std::vector<Pose2> gt = groundTruthPath(1.0);
  for (size_t i = 0; i < gt.size(); i += 2) {
    KeyPoseWithCloud kp;
    kp.r_global = Eigen::AngleAxisd(gt[i].yaw, V3D::UnitZ()).toRotationMatrix();
    kp.t_global = V3D(gt[i].x, gt[i].y, kSensorHeight);
    kp.body_cloud = w.scan(gt[i], rng);
    ret.push_back(kp);
  }
  return ret;
}

const std::vector<KeyPoseWithCloud> & saved()
{
  static const std::vector<KeyPoseWithCloud> map = savedMap(world());
  return map;
}

const std::vector<KeyPoseWithCloud> & savedSymmetric()
{
  static const std::vector<KeyPoseWithCloud> map = savedMap(symmetricWorld());
  return map;
}

// The body in the odometry frame: a little off the origin and turned, and pitched like the
// lidar, so nothing in the aligner can lean on "odom is the body".
const double kOdomYaw = 0.25;
const V3D kOdomPosition(0.3, -0.2, 0.05);

struct Stand
{
  SessionAlignRequest request;
  M3D r_true;  // map <- odom
  V3D t_true;
};

// The robot standing at `truth` in `w`. odom_tilt turns the whole odometry frame about its x
// axis: a LIO whose up is not the saved map's.
Stand standAt(const World & w, const Pose2 & truth, double odom_tilt = 0.0)
{
  std::mt19937 rng(41);
  Stand stand;
  const M3D r_tilt = Eigen::AngleAxisd(odom_tilt, V3D::UnitX()).toRotationMatrix();
  const M3D r_odom_yaw = Eigen::AngleAxisd(kOdomYaw, V3D::UnitZ()).toRotationMatrix();
  stand.request.r_body =
    r_tilt * r_odom_yaw * Eigen::AngleAxisd(15.0 * M_PI / 180.0, V3D::UnitY()).toRotationMatrix();
  stand.request.t_body = r_tilt * kOdomPosition;
  // The scan is level in the world, so in a level odom it is turned by the body's yaw only.
  stand.request.cloud_odom.reset(new CloudType);
  pcl::transformPointCloud(
    *w.scan(truth, rng), *stand.request.cloud_odom, stand.request.t_body,
    Eigen::Quaterniond(r_tilt * r_odom_yaw));

  // Out of the tilted odom into a level one, then about z onto the map.
  stand.r_true =
    Eigen::AngleAxisd(truth.yaw - kOdomYaw, V3D::UnitZ()).toRotationMatrix() * r_tilt.transpose();
  stand.t_true = V3D(truth.x, truth.y, kSensorHeight) - stand.r_true * stand.request.t_body;
  return stand;
}

void hint(Stand & stand, const Pose2 & at)
{
  stand.request.use_hint = true;
  stand.request.hint_position = V3D(at.x, at.y, kSensorHeight);
  stand.request.hint_yaw = at.yaw;
}

SessionAlignerConfig config()
{
  SessionAlignerConfig c;
  c.timeout_sec = 600.0;  // never the reason for a verdict here
  return c;
}

void print(const char * name, const SessionAlignResult & r)
{
  std::printf(
    "[%s] %s | %zu candidates, %zu registered | body at (%.2f, %.2f, %.2f) yaw %.1f deg | "
    "overlap %.2f inlier %.2f of scan %.2f fitness %.4f tilt %.2f deg%s%s\n",
    name, r.success ? "aligned" : "refused", r.candidates, r.attempts.size(), r.position.x(),
    r.position.y(), r.position.z(), r.yaw * 180.0 / M_PI, r.registration.overlap_ratio,
    r.registration.inlier_ratio, r.registration.source_inlier_ratio, r.registration.fitness,
    r.tilt_residual_deg, r.success ? "" : " | ", r.message.c_str());
  for (const SessionAlignAttempt & a : r.attempts)
    std::printf(
      "    at (%.2f, %.2f) yaw %.1f deg | coarse %.2f | inlier %.2f of scan %.2f tilt %.2f deg | "
      "%s%s\n",
      a.position.x(), a.position.y(), a.yaw * 180.0 / M_PI, a.coarse_score,
      a.registration.inlier_ratio, a.registration.source_inlier_ratio, a.tilt_residual_deg,
      a.passed ? "passed" : "rejected: ", a.reject_reason);
}

void expectAligned(const SessionAlignResult & r, const Stand & stand, const Pose2 & truth)
{
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_LT((r.t_offset - stand.t_true).norm(), 0.05);
  EXPECT_LT(
    Eigen::Quaterniond(r.r_offset).angularDistance(Eigen::Quaterniond(stand.r_true)),
    0.5 * M_PI / 180.0);
  EXPECT_LT((r.position - V3D(truth.x, truth.y, kSensorHeight)).norm(), 0.05);
  EXPECT_LT(std::abs(std::remainder(r.yaw - truth.yaw, 2.0 * M_PI)), 0.5 * M_PI / 180.0);
}

// The two frames share their up here, so the registration must not invent a tilt.
void expectLevel(const SessionAlignResult & r)
{
  EXPECT_LT(r.tilt_residual_deg, 0.2);
}
}  // namespace

TEST(SessionAligner, HintFindsThePose)
{
  const Pose2 truth{0.4, -7.6, 20.0 * M_PI / 180.0};
  Stand stand = standAt(world(), truth);
  hint(stand, Pose2{truth.x + 0.8, truth.y - 0.5, truth.yaw + 20.0 * M_PI / 180.0});
  SessionAligner aligner(config());
  const SessionAlignResult r = aligner.align(saved(), stand.request);
  print("hint", r);
  expectAligned(r, stand, truth);
  expectLevel(r);
  EXPECT_TRUE(r.used_hint);
}

// Three metres off the track the saved map was driven along: the target is the keyframes around
// the place, not the ones before and after some keyframe.
TEST(SessionAligner, HintFindsThePoseOffTheTrack)
{
  const Pose2 truth{2.0, -5.0, -100.0 * M_PI / 180.0};
  Stand stand = standAt(world(), truth);
  hint(stand, Pose2{truth.x - 1.0, truth.y + 0.6, truth.yaw - 15.0 * M_PI / 180.0});
  SessionAligner aligner(config());
  const SessionAlignResult r = aligner.align(saved(), stand.request);
  print("hint, off the track", r);
  expectAligned(r, stand, truth);
  expectLevel(r);
}

// A hint that is wrong is not an invitation to search: refused, not found somewhere else.
TEST(SessionAligner, FarHintIsRefused)
{
  const Pose2 truth{0.4, -7.6, 20.0 * M_PI / 180.0};
  Stand stand = standAt(world(), truth);
  hint(stand, Pose2{truth.x + 10.0, truth.y, truth.yaw});
  SessionAligner aligner(config());
  const SessionAlignResult r = aligner.align(saved(), stand.request);
  print("hint 10 m off", r);
  EXPECT_FALSE(r.success);
  EXPECT_FALSE(r.message.empty());
}

TEST(SessionAligner, HintOnAnotherStoreyIsRefused)
{
  const Pose2 truth{0.4, -7.6, 20.0 * M_PI / 180.0};
  Stand stand = standAt(world(), truth);
  hint(stand, truth);
  stand.request.hint_position.z() += 4.0;
  SessionAligner aligner(config());
  const SessionAlignResult r = aligner.align(saved(), stand.request);
  print("hint a storey up", r);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.attempts.empty());
}

TEST(SessionAligner, GlobalSearchFindsThePose)
{
  const Pose2 truth{-14.6, 3.0, 100.0 * M_PI / 180.0};
  const Stand stand = standAt(world(), truth);
  SessionAligner aligner(config());
  const SessionAlignResult r = aligner.align(saved(), stand.request);
  print("global", r);
  expectAligned(r, stand, truth);
  expectLevel(r);
  EXPECT_FALSE(r.used_hint);
}

// In the hall without furniture a place and its mirror image register equally well: the search
// must say so rather than pick one, and a hint must settle it.
TEST(SessionAligner, LookAlikePlaceIsRefusedWithoutAHint)
{
  const Pose2 truth{-14.6, 3.0, 100.0 * M_PI / 180.0};
  Stand stand = standAt(symmetricWorld(), truth);
  SessionAligner aligner(config());
  const SessionAlignResult global = aligner.align(savedSymmetric(), stand.request);
  print("symmetric hall, global", global);
  EXPECT_FALSE(global.success);
  EXPECT_NE(global.message.find("looks like another"), std::string::npos) << global.message;

  hint(stand, Pose2{truth.x + 0.5, truth.y + 0.5, truth.yaw + 10.0 * M_PI / 180.0});
  const SessionAlignResult hinted = aligner.align(savedSymmetric(), stand.request);
  print("symmetric hall, hint", hinted);
  expectAligned(hinted, stand, truth);
  expectLevel(hinted);
}

// A pose a localizer saved is a measurement: the robot is within centimetres of it, or it has
// been moved since and the pose is stale. Its window is tight, with none of an estimate's slack.
TEST(SessionAligner, MeasuredHintFindsThePoseItIsAt)
{
  const Pose2 truth{0.4, -7.6, 20.0 * M_PI / 180.0};
  Stand stand = standAt(world(), truth);
  hint(stand, Pose2{truth.x + 0.15, truth.y - 0.1, truth.yaw + 4.0 * M_PI / 180.0});
  stand.request.hint_radius = 1.0;
  stand.request.hint_yaw_deg = 20.0;
  SessionAligner aligner(config());
  const SessionAlignResult r = aligner.align(saved(), stand.request);
  print("measured hint", r);
  expectAligned(r, stand, truth);
  expectLevel(r);
}

// The robot 2.5 m from where the file says. As an operator's estimate that is a good hint and
// finds it; as a measurement it is a stale file, refused so that the caller searches instead.
TEST(SessionAligner, StaleMeasuredHintIsRefused)
{
  const Pose2 truth{0.4, -7.6, 20.0 * M_PI / 180.0};
  Stand stand = standAt(world(), truth);
  hint(stand, Pose2{truth.x + 2.5, truth.y, truth.yaw});
  SessionAligner aligner(config());
  const SessionAlignResult estimate = aligner.align(saved(), stand.request);
  print("2.5 m off, an estimate", estimate);
  expectAligned(estimate, stand, truth);

  stand.request.hint_radius = 1.0;
  stand.request.hint_yaw_deg = 20.0;
  const SessionAlignResult measured = aligner.align(saved(), stand.request);
  print("2.5 m off, a measurement", measured);
  EXPECT_FALSE(measured.success);
}

// An odometry frame whose up is 0.8 deg off the saved map's, as two static IMU initialisations
// leave two runs of one robot. The alignment is the whole transform, tilt included: the run takes
// the saved map's up. A rotation about z alone would leave it leaning by that much for good.
TEST(SessionAligner, TheRunsUpIsTurnedOntoTheSavedMaps)
{
  const Pose2 truth{0.4, -7.6, 20.0 * M_PI / 180.0};
  const double tilt = 0.8 * M_PI / 180.0;
  Stand stand = standAt(world(), truth, tilt);
  hint(stand, truth);
  SessionAligner aligner(config());
  const SessionAlignResult r = aligner.align(saved(), stand.request);
  print("tilted 0.8 deg", r);
  expectAligned(r, stand, truth);
  EXPECT_NEAR(r.tilt_residual_deg, 0.8, 0.15);
  // The saved map's up, seen from the odometry frame, is where the tilt put it.
  const V3D up_in_odom = r.r_offset.transpose() * V3D::UnitZ();
  const V3D up_true = stand.r_true.transpose() * V3D::UnitZ();
  EXPECT_LT(std::acos(std::min(1.0, up_in_odom.dot(up_true))), 0.15 * M_PI / 180.0);
  // A point 20 m from the robot lands where it is: 28 cm off with the tilt dropped.
  const V3D far(20.0, 0.0, 0.0);
  EXPECT_LT(((r.r_offset * far + r.t_offset) - (stand.r_true * far + stand.t_true)).norm(), 0.2);
}

// More tilt than two levellings can differ by is a registration gone wrong, not an up to adopt.
TEST(SessionAligner, TooMuchTiltIsRefused)
{
  const Pose2 truth{0.4, -7.6, 20.0 * M_PI / 180.0};
  Stand stand = standAt(world(), truth, 5.0 * M_PI / 180.0);
  hint(stand, truth);
  SessionAligner aligner(config());
  const SessionAlignResult r = aligner.align(saved(), stand.request);
  print("tilted 5 deg", r);
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("tilt"), std::string::npos) << r.message;

  SessionAlignerConfig loose = config();
  loose.max_tilt_deg = 8.0;
  SessionAligner loose_aligner(loose);
  const SessionAlignResult kept = loose_aligner.align(saved(), stand.request);
  print("tilted 5 deg, gate at 8", kept);
  ASSERT_TRUE(kept.success) << kept.message;
  EXPECT_NEAR(kept.tilt_residual_deg, 5.0, 0.3);
}

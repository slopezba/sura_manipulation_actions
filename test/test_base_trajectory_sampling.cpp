#include "sura_manipulation_actions/base_trajectory_sampling.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace sura_manipulation_actions
{
namespace
{

trajectory_msgs::msg::MultiDOFJointTrajectory trajectory(
  const BaseCoordinates & begin, const BaseCoordinates & finish,
  const BaseCoordinates & begin_velocity = BaseCoordinates::Zero(),
  const BaseCoordinates & finish_velocity = BaseCoordinates::Zero(),
  const BaseCoordinates & begin_acceleration = BaseCoordinates::Zero(),
  const BaseCoordinates & finish_acceleration = BaseCoordinates::Zero())
{
  trajectory_msgs::msg::MultiDOFJointTrajectory result;
  result.header.frame_id = "world_ned";
  result.joint_names = {"world_joint"};
  builtin_interfaces::msg::Duration zero, two;
  two.sec = 2;
  result.points.push_back(baseTrajectoryPoint(begin, begin_velocity, begin_acceleration, zero));
  result.points.push_back(baseTrajectoryPoint(finish, finish_velocity, finish_acceleration, two));
  return result;
}

TEST(BaseTrajectorySampling, PreservesFrozenEulerCoordinatesAndUnwrapsYaw)
{
  BaseCoordinates a, b;
  a << 0.0, 0.2, -0.3, 0.15, 0.1, 3.0;
  b << 0.5, 0.2, -0.3, 0.15, 0.2, 3.3;
  const auto message = trajectory(a, b);
  std::vector<BaseTrajectoryKnot> knots;
  std::string error;
  ASSERT_TRUE(prepareBaseTrajectory(message, knots, error)) << error;
  for (int i = 0; i <= 40; ++i) {
    BaseTrajectorySample sample;
    ASSERT_TRUE(sampleBaseTrajectory(knots, 2.0 * i / 40, sample, error)) << error;
    EXPECT_NEAR(sample.coordinates(1), a(1), 1.0e-12);
    EXPECT_NEAR(sample.coordinates(2), a(2), 1.0e-12);
    EXPECT_NEAR(sample.coordinates(3), a(3), 1.0e-12);
    EXPECT_GE(sample.coordinates(5), 3.0 - 1.0e-12);
    EXPECT_LE(sample.coordinates(5), 3.3 + 1.0e-12);
  }
  BaseTrajectorySample finish;
  ASSERT_TRUE(sampleBaseTrajectory(knots, 2.0, finish, error));
  EXPECT_NEAR(finish.coordinates(5), 3.3, 1.0e-12);
}

TEST(BaseTrajectorySampling, SpatialAngularDerivativesMatchPoseAndVelocity)
{
  BaseCoordinates a, b, va, vb, aa, ab;
  a << 0.0, 0.0, 0.0, 0.1, -0.2, 0.4;
  b << 0.1, -0.03, 0.2, 0.2, -0.1, 0.6;
  va << 0.02, 0.0, 0.0, 0.03, 0.04, -0.01;
  vb << 0.01, 0.01, 0.0, 0.02, 0.03, 0.04;
  aa << 0.01, 0.0, 0.0, -0.01, 0.02, 0.03;
  ab << 0.0, 0.0, 0.0, 0.02, -0.02, 0.01;
  const auto message = trajectory(a, b, va, vb, aa, ab);
  std::vector<BaseTrajectoryKnot> knots;
  std::string error;
  ASSERT_TRUE(prepareBaseTrajectory(message, knots, error)) << error;
  EXPECT_LT((knots.front().velocity - va).norm(), 1.0e-12);
  EXPECT_LT((knots.front().acceleration - aa).norm(), 1.0e-12);
  const double step = 1.0e-5;
  BaseTrajectorySample before, current, after;
  ASSERT_TRUE(sampleBaseTrajectory(knots, 0.7 - step, before, error));
  ASSERT_TRUE(sampleBaseTrajectory(knots, 0.7, current, error));
  ASSERT_TRUE(sampleBaseTrajectory(knots, 0.7 + step, after, error));
  const Eigen::Matrix3d derivative = (after.pose.rotation() - before.pose.rotation()) / (2.0 * step);
  const Eigen::Matrix3d spatial = derivative * current.pose.rotation().transpose();
  const Eigen::Vector3d omega(spatial(2, 1), spatial(0, 2), spatial(1, 0));
  EXPECT_LT((omega - baseAngularVector(current.twist)).norm(), 1.0e-8);
  const Eigen::Vector3d alpha =
    (baseAngularVector(after.twist) - baseAngularVector(before.twist)) / (2.0 * step);
  EXPECT_LT((alpha - baseAngularVector(current.twist_acceleration)).norm(), 1.0e-8);
}

TEST(BaseTrajectorySampling, KeepsContinuousRollWhenQuaternionCrossesPi)
{
  BaseCoordinates a = BaseCoordinates::Zero(), b = BaseCoordinates::Zero();
  a(3) = 3.0; b(3) = 3.3;
  const auto message = trajectory(a, b);
  BaseTrajectorySample sample;
  std::string error;
  ASSERT_TRUE(sampleBaseTrajectory(message, 1.0, sample, error)) << error;
  EXPECT_NEAR(sample.coordinates(3), 3.15, 1.0e-12);
  ASSERT_TRUE(sampleBaseTrajectory(message, 2.0, sample, error));
  EXPECT_NEAR(sample.coordinates(3), 3.3, 1.0e-12);
  EXPECT_LT((baseCoordinates(sample.pose, b) - b).norm(), 1.0e-12);
}

TEST(BaseTrajectorySampling, RejectsInvalidTimesQuaternionsAndSingularPitch)
{
  auto message = trajectory(BaseCoordinates::Zero(), BaseCoordinates::Zero());
  std::string error;
  message.points.back().time_from_start.sec = 0;
  EXPECT_FALSE(validateBaseTrajectory(message, error));
  message.points.back().time_from_start.sec = 2;
  message.points.front().transforms.front().rotation.w = 0.0;
  EXPECT_FALSE(validateBaseTrajectory(message, error));
  BaseCoordinates singular = BaseCoordinates::Zero();
  singular(4) = std::acos(-1.0) / 2.0;
  message = trajectory(singular, singular);
  EXPECT_FALSE(validateBaseTrajectory(message, error));
}

TEST(BaseTrajectorySampling, HoldsOutsideTheTrajectoryWithZeroFeedforward)
{
  BaseCoordinates a = BaseCoordinates::Zero(), b = BaseCoordinates::Zero();
  b(0) = 0.1;
  BaseCoordinates velocity = BaseCoordinates::Constant(0.01);
  const auto message = trajectory(a, b, velocity, velocity);
  BaseTrajectorySample sample;
  std::string error;
  ASSERT_TRUE(sampleBaseTrajectory(message, -0.5, sample, error));
  EXPECT_LT((sample.coordinates - a).norm(), 1.0e-12);
  EXPECT_DOUBLE_EQ(sample.velocity.norm(), 0.0);
  ASSERT_TRUE(sampleBaseTrajectory(message, 3.0, sample, error));
  EXPECT_LT((sample.coordinates - b).norm(), 1.0e-12);
  EXPECT_DOUBLE_EQ(sample.velocity.norm(), 0.0);
}

TEST(BaseTrajectorySampling, AdmissionRejectsPoseOvershootBetweenFeasibleKnots)
{
  const BaseCoordinates zero = BaseCoordinates::Zero();
  BaseCoordinates outgoing = zero, incoming = zero;
  outgoing(0) = 1.0; incoming(0) = -1.0;
  const auto message = trajectory(zero, zero, outgoing, incoming);
  std::vector<BaseTrajectoryKnot> knots;
  std::string error;
  ASSERT_TRUE(prepareBaseTrajectory(message, knots, error));
  EXPECT_FALSE(validateBaseTrajectoryLimits(knots, std::vector<double>(6, -0.1),
    std::vector<double>(6, 0.1), std::vector<double>(6, 2.0),
    std::vector<double>(6, 10.0), {}, error));
  EXPECT_NE(error.find("interpolation"), std::string::npos);
}

TEST(BaseTrajectorySampling, AdmissionChecksVelocityPeakAndLockedAxesBetweenPoints)
{
  BaseCoordinates begin = BaseCoordinates::Zero(), end = BaseCoordinates::Zero();
  end(0) = 1.0;
  std::vector<BaseTrajectoryKnot> knots;
  std::string error;
  ASSERT_TRUE(prepareBaseTrajectory(trajectory(begin, end), knots, error));
  EXPECT_FALSE(validateBaseTrajectoryLimits(knots, {}, {}, std::vector<double>(6, 0.5),
    std::vector<double>(6, 10.0), {}, error));
  EXPECT_NE(error.find("velocity"), std::string::npos);
  EXPECT_FALSE(validateBaseTrajectoryLimits(knots, {}, {}, std::vector<double>(6, 2.0),
    std::vector<double>(6, 10.0), std::vector<bool>(6, false), error));
  EXPECT_TRUE(validateBaseTrajectoryLimits(knots, {}, {}, std::vector<double>(6, 2.0),
    std::vector<double>(6, 10.0), std::vector<bool>(6, true), error)) << error;
}

}  // namespace
}  // namespace sura_manipulation_actions

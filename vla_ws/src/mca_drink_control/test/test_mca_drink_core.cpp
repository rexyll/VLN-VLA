#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "mca_drink_control/mca_arm_core.hpp"

namespace {

TEST(McaDrinkCore, DampedLeastSquaresSolvesReachableSerialChain) {
  using namespace mca::kinematics;
  DhParameter shoulder;
  shoulder.name = "shoulder";
  shoulder.type = JointType::Revolute;
  shoulder.a = 1.0;
  shoulder.limits.lower = -2.0;
  shoulder.limits.upper = 2.0;

  DhParameter elbow;
  elbow.name = "elbow";
  elbow.type = JointType::Revolute;
  elbow.a = 0.7;
  elbow.limits.lower = -2.0;
  elbow.limits.upper = 2.0;

  RobotModel model;
  ASSERT_TRUE(DhModelAdapter::build("test_arm", "base", {shoulder, elbow}, model));
  ForwardKinematicsResult target;
  ASSERT_TRUE(ForwardKinematics::solve(model, {0.5, -0.4}, target));

  const auto result = mca_drink_control::McaArmCore::solve_ik(
      model, target.pose, {0.0, 0.0});
  EXPECT_TRUE(result.converged) << result.message;
  EXPECT_LT(result.position_error_m, 1e-5);
  EXPECT_LT(result.orientation_error_rad, 1e-4);
  ASSERT_EQ(result.joint_positions.size(), 2U);
}

TEST(McaDrinkCore, SCurveRespectsBoundaryAndVelocityLimit) {
  mca_drink_control::McaJointSCurve trajectory(0.0, 1.0, 0.5, 1.0, 4.0);
  ASSERT_TRUE(trajectory.valid());
  EXPECT_DOUBLE_EQ(trajectory.sample(0.0).position, 0.0);
  const auto finish = trajectory.sample(trajectory.duration_s());
  EXPECT_DOUBLE_EQ(finish.position, 1.0);
  EXPECT_DOUBLE_EQ(finish.velocity, 0.0);
  for (int i = 0; i <= 1000; ++i) {
    const double t = trajectory.duration_s() * static_cast<double>(i) / 1000.0;
    EXPECT_LE(std::abs(trajectory.sample(t).velocity), 0.5 + 1e-9);
  }
}

TEST(McaDrinkCore, DifferentialDriveMapsTwistToWheelRates) {
  mca_drink_control::McaDifferentialDrive drive;
  ASSERT_TRUE(drive.configure(0.1, 0.5));
  double left = 0.0, right = 0.0;
  ASSERT_TRUE(drive.body_to_wheels(0.2, 0.4, left, right));
  EXPECT_NEAR(left, 1.0, 1e-12);
  EXPECT_NEAR(right, 3.0, 1e-12);
}

TEST(McaDrinkCore, RejectsInvalidDriveGeometry) {
  mca_drink_control::McaDifferentialDrive drive;
  EXPECT_FALSE(drive.configure(0.0, 0.5));
  double left = 0.0, right = 0.0;
  EXPECT_FALSE(drive.body_to_wheels(0.2, 0.0, left, right));
}

}  // namespace

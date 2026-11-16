#include <cmath>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
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

TEST(McaDrinkCore, ParsesGeneratedRightAndLeftArmModelsAndMatchesPythonFK) {
  using namespace mca::kinematics;
  const std::pair<const char*, const char*> paths[] = {
      {MCA_RIGHT_ARM_URDF_PATH, MCA_RIGHT_ARM_FK_PATH},
      {MCA_LEFT_ARM_URDF_PATH, MCA_LEFT_ARM_FK_PATH}};
  for (const auto& pair : paths) {
    std::ifstream urdf_file(pair.first);
    ASSERT_TRUE(urdf_file.good()) << pair.first;
    const std::string urdf((std::istreambuf_iterator<char>(urdf_file)),
                           std::istreambuf_iterator<char>());
    RobotModel model;
    std::string error;
    ASSERT_TRUE(UrdfModelAdapter::parse(urdf, model, &error)) << error;
    ASSERT_TRUE(model.valid(&error)) << error;
    ASSERT_EQ(model.joints.size(), 8U);
    ASSERT_EQ(model.links.size(), 9U);
    std::size_t movable = 0;
    for (const auto& joint : model.joints)
      if (joint.type != JointType::Fixed) ++movable;
    ASSERT_EQ(movable, 7U);

    std::ifstream reference(pair.second);
    ASSERT_TRUE(reference.good()) << pair.second;
    for (int sample = 0; sample < 3; ++sample) {
      std::vector<double> q(7);
      for (double& value : q) ASSERT_TRUE(static_cast<bool>(reference >> value));
      double expected[16]{};
      for (double& value : expected) ASSERT_TRUE(static_cast<bool>(reference >> value));
      ForwardKinematicsResult fk;
      ASSERT_TRUE(ForwardKinematics::solve(model, q, fk, &error)) << error;
      const auto rotation = mca::geometry::to_rotation_matrix(fk.pose.rotation);
      const double actual[16] = {
          rotation(0,0), rotation(0,1), rotation(0,2), fk.pose.position.x,
          rotation(1,0), rotation(1,1), rotation(1,2), fk.pose.position.y,
          rotation(2,0), rotation(2,1), rotation(2,2), fk.pose.position.z,
          0.0, 0.0, 0.0, 1.0};
      for (int i = 0; i < 16; ++i)
        EXPECT_NEAR(actual[i], expected[i], 2e-8)
            << "sample=" << sample << " element=" << i;
    }
  }
}

}  // namespace

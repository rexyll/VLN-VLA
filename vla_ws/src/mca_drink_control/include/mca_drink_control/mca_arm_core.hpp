#pragma once

#include <string>
#include <vector>

#include "mca/kinematics/damped_least_squares_ik.hpp"
#include "mca/kinematics/differential_drive.hpp"
#include "mca/trajectory/s_curve.hpp"

namespace mca_drink_control {

struct IkReport {
  bool converged{false};
  std::vector<double> joint_positions;
  std::size_t iterations{0};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  std::string message;
};

class McaArmCore {
 public:
  static IkReport solve_ik(
      const mca::kinematics::RobotModel& model,
      const mca::geometry::Pose3D& target,
      const std::vector<double>& seed,
      const mca::kinematics::DampedLeastSquaresIkOptions& options = {}) {
    const auto result =
        mca::kinematics::DampedLeastSquaresIk::solve(model, target, seed, options);
    IkReport report;
    report.converged = result.converged();
    report.joint_positions = result.joint_positions;
    report.iterations = result.iterations;
    report.position_error_m = result.position_error;
    report.orientation_error_rad = result.orientation_error;
    report.message = result.message;
    return report;
  }
};

class McaJointSCurve {
 public:
  McaJointSCurve(double start, double finish, double max_velocity,
                 double max_acceleration, double max_jerk)
      : trajectory_(start, finish, max_velocity, max_acceleration, max_jerk) {}

  bool valid() const noexcept { return trajectory_.valid(); }
  double duration_s() const noexcept { return trajectory_.duration(); }
  mca::trajectory::ScalarSample sample(double time_s) const noexcept {
    return trajectory_.sample(time_s);
  }

 private:
  mca::trajectory::SCurveTrajectory trajectory_;
};

class McaDifferentialDrive {
 public:
  bool configure(double wheel_radius_m, double track_width_m) noexcept {
    return model_.configure(wheel_radius_m, track_width_m);
  }
  bool body_to_wheels(double linear_x_mps, double angular_z_radps,
                      double& left_radps, double& right_radps) const noexcept {
    mca::kinematics::WheelSpeeds wheels;
    if (!model_.inverse({linear_x_mps, angular_z_radps}, wheels)) return false;
    left_radps = wheels.left;
    right_radps = wheels.right;
    return true;
  }

 private:
  mca::kinematics::DifferentialDriveKinematics model_;
};

}  // namespace mca_drink_control

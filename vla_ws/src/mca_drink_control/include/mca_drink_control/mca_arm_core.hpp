#pragma once

#include <cmath>
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

class McaSynchronizedJointSCurve {
 public:
  bool configure(const std::vector<double>& start,
                 const std::vector<double>& finish,
                 const std::vector<double>& max_velocity,
                 const std::vector<double>& max_acceleration,
                 const std::vector<double>& max_jerk) {
    valid_ = false;
    trajectories_.clear();
    duration_s_ = 0.0;
    const std::size_t n = start.size();
    if (n == 0 || finish.size() != n || max_velocity.size() != n ||
        max_acceleration.size() != n || max_jerk.size() != n)
      return false;

    std::vector<double> axis_durations(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      if (!std::isfinite(start[i]) || !std::isfinite(finish[i]) ||
          !std::isfinite(max_velocity[i]) || max_velocity[i] <= 0.0 ||
          !std::isfinite(max_acceleration[i]) || max_acceleration[i] <= 0.0 ||
          !std::isfinite(max_jerk[i]) || max_jerk[i] <= 0.0)
        return false;
      const mca::trajectory::SCurveTrajectory base(
          start[i], finish[i], max_velocity[i], max_acceleration[i], max_jerk[i]);
      if (!base.valid()) return false;
      axis_durations[i] = base.duration();
      duration_s_ = std::max(duration_s_, axis_durations[i]);
    }

    std::vector<mca::trajectory::SCurveTrajectory> candidate;
    candidate.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      if (axis_durations[i] <= 0.0 || duration_s_ <= 0.0) {
        candidate.emplace_back(start[i], finish[i], max_velocity[i],
                               max_acceleration[i], max_jerk[i]);
        continue;
      }
      const double scale = duration_s_ / axis_durations[i];
      const double velocity = max_velocity[i] / scale;
      const double acceleration = max_acceleration[i] / (scale * scale);
      const double jerk = max_jerk[i] / (scale * scale * scale);
      candidate.emplace_back(start[i], finish[i], velocity, acceleration, jerk);
      const double duration_error =
          std::abs(candidate.back().duration() - duration_s_);
      if (!candidate.back().valid() ||
          duration_error > 1e-8 * std::max(1.0, duration_s_))
        return false;
    }
    trajectories_ = std::move(candidate);
    valid_ = true;
    return true;
  }

  bool valid() const noexcept { return valid_; }
  std::size_t joint_count() const noexcept { return trajectories_.size(); }
  double duration_s() const noexcept { return duration_s_; }

  bool sample(double time_s,
              std::vector<mca::trajectory::ScalarSample>& output) const {
    if (!valid_ || !std::isfinite(time_s)) return false;
    std::vector<mca::trajectory::ScalarSample> candidate;
    candidate.reserve(trajectories_.size());
    for (const auto& trajectory : trajectories_) {
      const auto value = trajectory.sample(time_s);
      if (!std::isfinite(value.position) || !std::isfinite(value.velocity) ||
          !std::isfinite(value.acceleration))
        return false;
      candidate.push_back(value);
    }
    output = std::move(candidate);
    return true;
  }

 private:
  std::vector<mca::trajectory::SCurveTrajectory> trajectories_;
  double duration_s_{0.0};
  bool valid_{false};
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

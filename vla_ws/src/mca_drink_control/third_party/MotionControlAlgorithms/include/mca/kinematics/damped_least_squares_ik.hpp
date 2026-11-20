#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include "mca/kinematics/numerical_ik.hpp"

namespace mca::kinematics {

struct DampedLeastSquaresIkOptions {
  std::size_t max_iterations{200};
  double position_tolerance{1e-5};
  double orientation_tolerance{1e-4};
  double finite_difference_step{1e-6};
  double initial_damping{1e-2};
  double min_damping{1e-6};
  double max_damping{1e3};
  double damping_increase{10.0};
  double damping_decrease{0.3};
  double max_joint_step{0.2};
  std::size_t max_trials_per_iteration{8};
  bool enforce_joint_limits{true};
};

class DampedLeastSquaresIk {
 public:
  static NumericalIkResult solve(const RobotModel& model,
                                 const geometry::Pose3D& target,
                                 const std::vector<double>& initial,
                                 const DampedLeastSquaresIkOptions& options = {}) {
    NumericalIkResult result;
    result.joint_positions = initial;
    std::string model_error;
    if (!model.valid(&model_error)) return invalid(result, model_error);
    std::size_t n = 0;
    for (const auto& joint : model.joints) if (joint.type != JointType::Fixed) ++n;
    if (initial.size() != n) return invalid(result, "initial coordinate count does not match movable joints");
    if (!finite_pose(target) || !finite_vector(initial) || options.max_iterations == 0 ||
        options.max_trials_per_iteration == 0 || !positive(options.position_tolerance) ||
        !positive(options.orientation_tolerance) || !positive(options.finite_difference_step) ||
        !positive(options.initial_damping) || !positive(options.min_damping) ||
        !positive(options.max_damping) || options.min_damping > options.initial_damping ||
        options.initial_damping > options.max_damping || !std::isfinite(options.damping_increase) ||
        options.damping_increase <= 1.0 || !positive(options.damping_decrease) ||
        options.damping_decrease >= 1.0 || !positive(options.max_joint_step))
      return invalid(result, "invalid target, initial coordinates, or DLS options");

    std::vector<double> q = initial;
    double damping = options.initial_damping;
    for (std::size_t iteration = 0; iteration <= options.max_iterations; ++iteration) {
      ForwardKinematicsResult fk;
      if (!ForwardKinematics::solve(model, q, fk, &result.message)) {
        result.status = IkStatus::InvalidInput;
        return result;
      }
      const auto e = error(target, fk.pose);
      const double current_cost = squared_norm(e);
      update_residuals(result, e);
      result.iterations = iteration;
      result.joint_positions = q;
      if (result.position_error <= options.position_tolerance &&
          result.orientation_error <= options.orientation_tolerance) {
        result.status = IkStatus::Converged;
        result.message = "converged";
        return result;
      }
      if (iteration == options.max_iterations) break;

      std::vector<std::array<double, 6>> columns(n);
      for (std::size_t j = 0; j < n; ++j) {
        auto perturbed = q;
        perturbed[j] += options.finite_difference_step;
        ForwardKinematicsResult next;
        if (!ForwardKinematics::solve(model, perturbed, next, &result.message)) {
          result.status = IkStatus::InvalidInput;
          return result;
        }
        const auto delta = error(next.pose, fk.pose);
        for (std::size_t r = 0; r < 6; ++r)
          columns[j][r] = delta[r] / options.finite_difference_step;
      }

      bool accepted = false;
      for (std::size_t trial = 0; trial < options.max_trials_per_iteration; ++trial) {
        std::array<std::array<double, 6>, 6> a{};
        for (std::size_t r = 0; r < 6; ++r) {
          for (std::size_t c = 0; c < 6; ++c)
            for (std::size_t j = 0; j < n; ++j)
              a[r][c] += columns[j][r] * columns[j][c];
          a[r][r] += damping * damping;
        }
        std::array<double, 6> y = e;
        if (!solve_linear(a, y)) {
          damping = std::min(options.max_damping, damping * options.damping_increase);
          continue;
        }
        auto candidate = q;
        for (std::size_t j = 0; j < n; ++j) {
          double dq = 0.0;
          for (std::size_t r = 0; r < 6; ++r) dq += columns[j][r] * y[r];
          dq = std::max(-options.max_joint_step, std::min(options.max_joint_step, dq));
          candidate[j] += dq;
          if (options.enforce_joint_limits) {
            const auto limit = movable_limit(model, j);
            candidate[j] = std::max(limit.first, std::min(limit.second, candidate[j]));
          }
        }
        ForwardKinematicsResult candidate_fk;
        if (!ForwardKinematics::solve(model, candidate, candidate_fk, &result.message)) {
          result.status = IkStatus::InvalidInput;
          return result;
        }
        const double candidate_cost = squared_norm(error(target, candidate_fk.pose));
        if (std::isfinite(candidate_cost) && candidate_cost < current_cost - 1e-15) {
          q = std::move(candidate);
          damping = std::max(options.min_damping, damping * options.damping_decrease);
          accepted = true;
          break;
        }
        if (damping >= options.max_damping) break;
        damping = std::min(options.max_damping, damping * options.damping_increase);
      }
      if (!accepted) {
        result.status = IkStatus::UnreachableOrStalled;
        result.message = "DLS could not find a cost-reducing step within the trial budget";
        result.joint_positions = q;
        return result;
      }
    }
    result.status = IkStatus::MaxIterations;
    result.message = "maximum iteration count reached";
    result.joint_positions = q;
    return result;
  }

 private:
  static NumericalIkResult invalid(NumericalIkResult result, const std::string& message) {
    result.status = IkStatus::InvalidInput;
    result.message = message;
    return result;
  }
  static bool positive(double x) { return std::isfinite(x) && x > 0.0; }
  static bool finite_vector(const std::vector<double>& values) {
    for (double value : values) if (!std::isfinite(value)) return false;
    return true;
  }
  static bool finite_pose(const geometry::Pose3D& pose) {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z) && std::isfinite(pose.rotation.w) &&
      std::isfinite(pose.rotation.x) && std::isfinite(pose.rotation.y) &&
      std::isfinite(pose.rotation.z) && geometry::quaternion_norm(pose.rotation) > 1e-12;
  }
  static std::array<double, 6> error(const geometry::Pose3D& target,
                                     const geometry::Pose3D& current) {
    const auto dp = target.position - current.position;
    const auto dq = geometry::normalized(geometry::multiply(
      target.rotation, geometry::conjugate(geometry::normalized(current.rotation))));
    const auto dr = geometry::log_so3(dq);
    return {dp.x, dp.y, dp.z, dr.x, dr.y, dr.z};
  }
  static double squared_norm(const std::array<double, 6>& values) {
    double sum = 0.0;
    for (double value : values) sum += value * value;
    return sum;
  }
  static void update_residuals(NumericalIkResult& result, const std::array<double, 6>& e) {
    result.position_error = std::sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
    result.orientation_error = std::sqrt(e[3]*e[3] + e[4]*e[4] + e[5]*e[5]);
  }
  static bool solve_linear(std::array<std::array<double, 6>, 6>& a,
                           std::array<double, 6>& b) {
    for (std::size_t c = 0; c < 6; ++c) {
      std::size_t pivot = c;
      for (std::size_t r = c + 1; r < 6; ++r)
        if (std::abs(a[r][c]) > std::abs(a[pivot][c])) pivot = r;
      if (std::abs(a[pivot][c]) < 1e-18) return false;
      std::swap(a[pivot], a[c]);
      std::swap(b[pivot], b[c]);
      const double diagonal = a[c][c];
      for (std::size_t k = c; k < 6; ++k) a[c][k] /= diagonal;
      b[c] /= diagonal;
      for (std::size_t r = 0; r < 6; ++r) if (r != c) {
        const double factor = a[r][c];
        for (std::size_t k = c; k < 6; ++k) a[r][k] -= factor * a[c][k];
        b[r] -= factor * b[c];
      }
    }
    return true;
  }
  static std::pair<double, double> movable_limit(const RobotModel& model, std::size_t index) {
    std::size_t current = 0;
    for (const auto& joint : model.joints) if (joint.type != JointType::Fixed) {
      if (current++ == index) return {joint.limits.lower, joint.limits.upper};
    }
    return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }
};

}  // namespace mca::kinematics

#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include "mca/kinematics/forward_kinematics.hpp"

namespace mca::kinematics {

enum class IkStatus { Converged, InvalidInput, UnreachableOrStalled, MaxIterations };

struct NumericalIkOptions {
  std::size_t max_iterations{200};
  double position_tolerance{1e-5};
  double orientation_tolerance{1e-4};
  double finite_difference_step{1e-6};
  double damping{1e-4};
  double max_joint_step{0.2};
  bool enforce_joint_limits{true};
};

struct NumericalIkResult {
  IkStatus status{IkStatus::InvalidInput};
  std::vector<double> joint_positions{};
  std::size_t iterations{0};
  double position_error{std::numeric_limits<double>::infinity()};
  double orientation_error{std::numeric_limits<double>::infinity()};
  std::string message{};
  bool converged() const noexcept { return status == IkStatus::Converged; }
};

class NumericalIk {
 public:
  static NumericalIkResult solve(const RobotModel& model,
                                const geometry::Pose3D& target,
                                const std::vector<double>& initial,
                                const NumericalIkOptions& options = {}) {
    NumericalIkResult result;
    result.joint_positions = initial;
    std::string model_error;
    if (!model.valid(&model_error)) return invalid(result, model_error);
    std::size_t n = 0;
    for (const auto& joint : model.joints) if (joint.type != JointType::Fixed) ++n;
    if (initial.size() != n) return invalid(result, "initial coordinate count does not match movable joints");
    if (!finite_pose(target) || !finite_vector(initial) ||
        options.max_iterations == 0 || !positive_finite(options.position_tolerance) ||
        !positive_finite(options.orientation_tolerance) ||
        !positive_finite(options.finite_difference_step) ||
        !positive_finite(options.damping) || !positive_finite(options.max_joint_step))
      return invalid(result, "invalid target, initial coordinates, or solver options");

    std::vector<double> q = initial;
    for (std::size_t iteration = 0; iteration <= options.max_iterations; ++iteration) {
      ForwardKinematicsResult fk;
      if (!ForwardKinematics::solve(model, q, fk, &result.message)) {
        result.status = IkStatus::InvalidInput;
        return result;
      }
      const auto e = pose_error(target, fk.pose);
      result.iterations = iteration;
      result.position_error = geometry::norm({e[0], e[1], e[2]});
      result.orientation_error = geometry::norm({e[3], e[4], e[5]});
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
        std::vector<double> perturbed = q;
        perturbed[j] += options.finite_difference_step;
        ForwardKinematicsResult next;
        if (!ForwardKinematics::solve(model, perturbed, next, &result.message)) {
          result.status = IkStatus::InvalidInput;
          return result;
        }
        const auto delta = pose_error(next.pose, fk.pose);
        for (std::size_t r = 0; r < 6; ++r)
          columns[j][r] = delta[r] / options.finite_difference_step;
      }

      // Solve (J J^T + lambda^2 I)y=e, then dq=J^T y.
      std::array<std::array<double, 6>, 6> a{};
      for (std::size_t r = 0; r < 6; ++r) {
        for (std::size_t c = 0; c < 6; ++c) {
          for (std::size_t j = 0; j < n; ++j) a[r][c] += columns[j][r] * columns[j][c];
        }
        a[r][r] += options.damping * options.damping;
      }
      std::array<double, 6> y = e;
      if (!solve_linear(a, y)) {
        result.status = IkStatus::UnreachableOrStalled;
        result.message = "numerical Jacobian system is singular";
        return result;
      }
      bool moved = false;
      for (std::size_t j = 0; j < n; ++j) {
        double dq = 0;
        for (std::size_t r = 0; r < 6; ++r) dq += columns[j][r] * y[r];
        dq = std::max(-options.max_joint_step, std::min(options.max_joint_step, dq));
        const double old = q[j];
        q[j] += dq;
        if (options.enforce_joint_limits) {
          const auto limit = movable_limit(model, j);
          q[j] = std::max(limit.first, std::min(limit.second, q[j]));
        }
        moved = moved || std::abs(q[j] - old) > 1e-14;
      }
      if (!moved) {
        result.status = IkStatus::UnreachableOrStalled;
        result.message = "solver stalled at joint limits or under a negligible update";
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
  static NumericalIkResult invalid(NumericalIkResult r, const std::string& m) {
    r.status = IkStatus::InvalidInput; r.message = m; return r;
  }
  static bool positive_finite(double x) { return std::isfinite(x) && x > 0; }
  static bool finite_vector(const std::vector<double>& v) {
    for (double x : v) if (!std::isfinite(x)) return false;
    return true;
  }
  static bool finite_pose(const geometry::Pose3D& p) {
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
      std::isfinite(p.position.z) && std::isfinite(p.rotation.w) &&
      std::isfinite(p.rotation.x) && std::isfinite(p.rotation.y) &&
      std::isfinite(p.rotation.z) && geometry::quaternion_norm(p.rotation) > 1e-12;
  }
  static std::array<double, 6> pose_error(const geometry::Pose3D& target,
                                          const geometry::Pose3D& current) {
    const auto dp = target.position - current.position;
    const auto dq = geometry::normalized(geometry::multiply(
        target.rotation, geometry::conjugate(geometry::normalized(current.rotation))));
    const auto dr = geometry::log_so3(dq);
    return {dp.x, dp.y, dp.z, dr.x, dr.y, dr.z};
  }
  static bool solve_linear(std::array<std::array<double, 6>, 6>& a,
                           std::array<double, 6>& b) {
    for (std::size_t c = 0; c < 6; ++c) {
      std::size_t pivot = c;
      for (std::size_t r = c + 1; r < 6; ++r)
        if (std::abs(a[r][c]) > std::abs(a[pivot][c])) pivot = r;
      if (std::abs(a[pivot][c]) < 1e-15) return false;
      std::swap(a[pivot], a[c]); std::swap(b[pivot], b[c]);
      const double d = a[c][c];
      for (std::size_t k = c; k < 6; ++k) a[c][k] /= d;
      b[c] /= d;
      for (std::size_t r = 0; r < 6; ++r) if (r != c) {
        const double f = a[r][c];
        for (std::size_t k = c; k < 6; ++k) a[r][k] -= f * a[c][k];
        b[r] -= f * b[c];
      }
    }
    return true;
  }
  static std::pair<double, double> movable_limit(const RobotModel& model, std::size_t index) {
    std::size_t k = 0;
    for (const auto& j : model.joints) if (j.type != JointType::Fixed) {
      if (k++ == index) return {j.limits.lower, j.limits.upper};
    }
    return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }
};

}  // namespace mca::kinematics

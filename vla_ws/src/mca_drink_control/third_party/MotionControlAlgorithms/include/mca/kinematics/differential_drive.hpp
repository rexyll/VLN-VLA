#pragma once

#include <cmath>
#include <limits>

namespace mca::kinematics {

struct Twist2D {
  double linear_x{0.0};
  double angular_z{0.0};
};

struct WheelSpeeds {
  double left{0.0};
  double right{0.0};
};

class DifferentialDriveKinematics {
 public:
  DifferentialDriveKinematics() = default;

  DifferentialDriveKinematics(double wheel_radius, double track_width) {
    configure(wheel_radius, track_width);
  }

  bool configure(double wheel_radius, double track_width) noexcept {
    if (!std::isfinite(wheel_radius) || !std::isfinite(track_width) ||
        wheel_radius <= 0.0 || track_width <= 0.0) {
      configured_ = false;
      return false;
    }
    wheel_radius_ = wheel_radius;
    track_width_ = track_width;
    configured_ = true;
    return true;
  }

  bool configured() const noexcept { return configured_; }

  double wheel_radius() const noexcept { return wheel_radius_; }
  double track_width() const noexcept { return track_width_; }

  // Map body twist (m/s, rad/s) to left/right wheel angular velocity (rad/s).
  bool inverse(const Twist2D& twist, WheelSpeeds& wheels) const noexcept {
    if (!configured_ || !finite(twist)) return false;
    const double half_track = 0.5 * track_width_;
    wheels.left = (twist.linear_x - half_track * twist.angular_z) /
                  wheel_radius_;
    wheels.right = (twist.linear_x + half_track * twist.angular_z) /
                   wheel_radius_;
    return finite(wheels);
  }

  // Map left/right wheel angular velocity (rad/s) to body twist (m/s, rad/s).
  bool forward(const WheelSpeeds& wheels, Twist2D& twist) const noexcept {
    if (!configured_ || !finite(wheels)) return false;
    twist.linear_x = 0.5 * wheel_radius_ * (wheels.left + wheels.right);
    twist.angular_z = wheel_radius_ * (wheels.right - wheels.left) /
                      track_width_;
    return finite(twist);
  }

 private:
  static bool finite(const Twist2D& value) noexcept {
    return std::isfinite(value.linear_x) && std::isfinite(value.angular_z);
  }

  static bool finite(const WheelSpeeds& value) noexcept {
    return std::isfinite(value.left) && std::isfinite(value.right);
  }

  double wheel_radius_{0.0};
  double track_width_{0.0};
  bool configured_{false};
};

}  // namespace mca::kinematics

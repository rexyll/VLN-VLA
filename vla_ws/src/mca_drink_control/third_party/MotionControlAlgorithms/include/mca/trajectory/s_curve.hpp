#pragma once

#include <array>
#include <cmath>

#include "mca/trajectory/scalar.hpp"

namespace mca::trajectory {

// Time-optimal, symmetric, rest-to-rest S-curve under velocity, acceleration
// and jerk limits. Position units are caller-defined; time is seconds.
class SCurveTrajectory final : public ISampledScalarTrajectory {
 public:
  SCurveTrajectory(double start_position, double finish_position,
                   double max_velocity, double max_acceleration,
                   double max_jerk) noexcept
      : start_position_(start_position), finish_position_(finish_position),
        distance_(std::abs(finish_position - start_position)),
        direction_(finish_position >= start_position ? 1.0 : -1.0),
        max_velocity_(max_velocity), max_acceleration_(max_acceleration),
        max_jerk_(max_jerk) {
    valid_ = std::isfinite(start_position_) && std::isfinite(finish_position_) &&
             std::isfinite(max_velocity_) && std::isfinite(max_acceleration_) &&
             std::isfinite(max_jerk_) && max_velocity_ > 0.0 &&
             max_acceleration_ > 0.0 && max_jerk_ > 0.0;
    if (!valid_ || distance_ == 0.0) return;

    const double tj_accel_limit = max_acceleration_ / max_jerk_;
    const double velocity_at_accel_limit =
        max_acceleration_ * max_acceleration_ / max_jerk_;
    if (max_velocity_ >= velocity_at_accel_limit) {
      jerk_time_ = tj_accel_limit;
      accel_hold_time_ = max_velocity_ / max_acceleration_ - jerk_time_;
    } else {
      jerk_time_ = std::sqrt(max_velocity_ / max_jerk_);
      accel_hold_time_ = 0.0;
    }

    const double accel_phase_time = 2.0 * jerk_time_ + accel_hold_time_;
    const double distance_at_max_velocity = max_velocity_ * accel_phase_time;
    if (distance_ >= distance_at_max_velocity) {
      cruise_time_ = (distance_ - distance_at_max_velocity) / max_velocity_;
    } else {
      cruise_time_ = 0.0;
      const double distance_to_accel_limit =
          2.0 * max_jerk_ * tj_accel_limit * tj_accel_limit * tj_accel_limit;
      if (max_velocity_ >= velocity_at_accel_limit &&
          distance_ >= distance_to_accel_limit) {
        jerk_time_ = tj_accel_limit;
        const double discriminant =
            tj_accel_limit * tj_accel_limit +
            4.0 * distance_ / (max_jerk_ * tj_accel_limit);
        accel_hold_time_ =
            0.5 * (std::sqrt(discriminant) - 3.0 * tj_accel_limit);
        if (accel_hold_time_ < 0.0) accel_hold_time_ = 0.0;
      } else {
        jerk_time_ = std::cbrt(distance_ / (2.0 * max_jerk_));
        accel_hold_time_ = 0.0;
      }
    }
    durations_ = {jerk_time_, accel_hold_time_, jerk_time_, cruise_time_,
                  jerk_time_, accel_hold_time_, jerk_time_};
    jerks_ = {max_jerk_, 0.0, -max_jerk_, 0.0,
              -max_jerk_, 0.0, max_jerk_};
    for (double duration : durations_) total_duration_ += duration;
  }

  bool valid() const noexcept { return valid_; }
  double duration() const noexcept { return total_duration_; }

  ScalarSample sample(double time_s) const noexcept override {
    if (!valid_) return {start_position_, 0.0, 0.0};
    if (distance_ == 0.0 || time_s <= 0.0) return {start_position_, 0.0, 0.0};
    if (time_s >= total_duration_) return {finish_position_, 0.0, 0.0};

    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    double remaining = time_s;
    for (std::size_t i = 0; i < durations_.size(); ++i) {
      const double dt = remaining < durations_[i] ? remaining : durations_[i];
      const double jerk = jerks_[i];
      position += velocity * dt + 0.5 * acceleration * dt * dt +
                  jerk * dt * dt * dt / 6.0;
      velocity += acceleration * dt + 0.5 * jerk * dt * dt;
      acceleration += jerk * dt;
      remaining -= dt;
      if (remaining <= 0.0) break;
    }
    return {start_position_ + direction_ * position,
            direction_ * velocity, direction_ * acceleration};
  }

 private:
  double start_position_{0.0};
  double finish_position_{0.0};
  double distance_{0.0};
  double direction_{1.0};
  double max_velocity_{0.0};
  double max_acceleration_{0.0};
  double max_jerk_{0.0};
  double jerk_time_{0.0};
  double accel_hold_time_{0.0};
  double cruise_time_{0.0};
  double total_duration_{0.0};
  bool valid_{false};
  std::array<double, 7> durations_{};
  std::array<double, 7> jerks_{};
};

}  // namespace mca::trajectory

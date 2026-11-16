#pragma once

#include <cmath>

namespace mca::trajectory {

struct ScalarSample {
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

class ISampledScalarTrajectory {
 public:
  virtual ~ISampledScalarTrajectory() = default;
  virtual ScalarSample sample(double time_s) const noexcept = 0;
};

class StepTrajectory final : public ISampledScalarTrajectory {
 public:
  explicit StepTrajectory(double target, double initial = 0.0) noexcept
      : initial_(initial), target_(target) {}

  ScalarSample sample(double time_s) const noexcept override {
    return {time_s < 0.0 ? initial_ : target_, 0.0, 0.0};
  }

 private:
  double initial_{0.0};
  double target_{0.0};
};

class RampTrajectory final : public ISampledScalarTrajectory {
 public:
  RampTrajectory(double start, double finish, double duration_s) noexcept
      : start_(start), finish_(finish), duration_s_(duration_s) {}

  ScalarSample sample(double time_s) const noexcept override {
    if (!(duration_s_ > 0.0)) return {finish_, 0.0, 0.0};
    if (time_s <= 0.0) return {start_, 0.0, 0.0};
    if (time_s >= duration_s_) return {finish_, 0.0, 0.0};
    const double velocity = (finish_ - start_) / duration_s_;
    return {start_ + velocity * time_s, velocity, 0.0};
  }

 private:
  double start_{0.0};
  double finish_{0.0};
  double duration_s_{0.0};
};

class SineTrajectory final : public ISampledScalarTrajectory {
 public:
  SineTrajectory(double amplitude, double frequency_hz, double offset = 0.0,
                 double phase_rad = 0.0) noexcept
      : amplitude_(amplitude), frequency_hz_(frequency_hz), offset_(offset),
        phase_rad_(phase_rad) {}

  ScalarSample sample(double time_s) const noexcept override {
    constexpr double kTwoPi = 6.28318530717958647692;
    const double omega = kTwoPi * frequency_hz_;
    const double phase = omega * time_s + phase_rad_;
    return {offset_ + amplitude_ * std::sin(phase),
            amplitude_ * omega * std::cos(phase),
            -amplitude_ * omega * omega * std::sin(phase)};
  }

 private:
  double amplitude_{0.0};
  double frequency_hz_{0.0};
  double offset_{0.0};
  double phase_rad_{0.0};
};

}  // namespace mca::trajectory

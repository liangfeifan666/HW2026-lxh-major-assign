/**
 *******************************************************************************
 * @file      : pid.cpp
 * @brief     : 1ms 周期 PID；异常输入与饱和时均不会产生无界输出
 *******************************************************************************
 */
#include "pid.hpp"

#include <algorithm>
#include <cmath>

namespace hello_world {
namespace pid {

Pid::Pid(float kp, float ki, float kd, float integral_limit,
         float output_limit, float sample_time)
    : kp_(kp), ki_(ki), kd_(kd),
      integral_limit_(std::fabs(integral_limit)),
      output_limit_(std::fabs(output_limit)),
      sample_time_(sample_time) {}

float Pid::calculate(float target, float feedback) {
  if (!std::isfinite(target) || !std::isfinite(feedback) ||
      sample_time_ <= 0.0f) {
    reset();
    return 0.0f;
  }

  const float error = target - feedback;
  const float old_integral = integral_;
  integral_ = std::clamp(integral_ + error * sample_time_,
                         -integral_limit_, integral_limit_);
  const float derivative = (error - previous_error_) / sample_time_;
  previous_error_ = error;

  float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  if (std::fabs(output) > output_limit_) {
    // 饱和且误差仍把输出推向同一方向时，撤销本次积分。
    if (error * output > 0.0f) {
      integral_ = old_integral;
      output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    }
    output = std::clamp(output, -output_limit_, output_limit_);
  }
  return std::isfinite(output) ? output : 0.0f;
}

void Pid::reset() {
  integral_ = 0.0f;
  previous_error_ = 0.0f;
}

}  // namespace pid
}  // namespace hello_world

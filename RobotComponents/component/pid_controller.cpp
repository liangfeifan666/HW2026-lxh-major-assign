/**
 *******************************************************************************
 * @file      : pid_controller.cpp
 * @brief     : 首拍零微分、一阶微分滤波、积分限幅与同向抗饱和
 *******************************************************************************
 */
#include "pid_controller.hpp"

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
  const float derivative = has_previous_error_
                               ? (error - previous_error_) / sample_time_
                               : 0.0f;
  return calculateImpl(target, feedback, derivative);
}

float Pid::calculate(float target, float feedback, float error_derivative) {
  if (!std::isfinite(target) || !std::isfinite(feedback) ||
      !std::isfinite(error_derivative) || sample_time_ <= 0.0f) {
    reset();
    return 0.0f;
  }
  return calculateImpl(target, feedback, error_derivative);
}

float Pid::calculateImpl(float target, float feedback, float error_derivative) {
  const float error = target - feedback;
  const float old_integral = integral_;
  integral_ = std::clamp(integral_ + error * sample_time_,
                         -integral_limit_, integral_limit_);

  // 复位后的首拍不微分；之后 0.8/0.2 低通抑制反馈噪声与偶发跳变。
  derivative_state_ = has_previous_error_
                          ? 0.8f * derivative_state_ + 0.2f * error_derivative
                          : 0.0f;
  previous_error_ = error;
  has_previous_error_ = true;

  float output = kp_ * error + ki_ * integral_ + kd_ * derivative_state_;
  if (std::fabs(output) > output_limit_) {
    if (error * output > 0.0f) {
      integral_ = old_integral;
      output = kp_ * error + ki_ * integral_ + kd_ * derivative_state_;
    }
    output = std::clamp(output, -output_limit_, output_limit_);
  }
  return std::isfinite(output) ? output : 0.0f;
}

void Pid::reset() {
  integral_ = 0.0f;
  previous_error_ = 0.0f;
  derivative_state_ = 0.0f;
  has_previous_error_ = false;
}

}  // namespace pid
}  // namespace hello_world

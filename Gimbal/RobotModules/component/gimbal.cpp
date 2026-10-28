/**
 *******************************************************************************
 * @file      : gimbal.cpp
 * @brief     : yaw 最短角误差、pitch 目标/反馈双限位和位置环输出
 *******************************************************************************
 */
#include "gimbal.hpp"

#include <cmath>

namespace robot {

void Gimbal::registerYaw(hello_world::motor::Motor *motor,
                         hello_world::pid::Pid *pid) {
  yaw_motor_ = motor;
  yaw_pid_ = pid;
}

void Gimbal::registerPitch(hello_world::motor::Motor *motor,
                           hello_world::pid::Pid *pid) {
  pitch_motor_ = motor;
  pitch_pid_ = pid;
}

GimbalMotion Gimbal::feedback() const {
  return {yaw_motor_ ? WrapPi(yaw_motor_->angle()) : 0.0f,
          pitch_motor_ ? WrapPi(pitch_motor_->angle()) : 0.0f};
}

void Gimbal::resetTarget() {
  target_ = feedback();
  if (yaw_pid_) yaw_pid_->reset();
  if (pitch_pid_) pitch_pid_->reset();
}

void Gimbal::setTarget(const GimbalMotion &target) {
  if (!std::isfinite(target.yaw) || !std::isfinite(target.pitch)) {
    stop();
    return;
  }
  target_.yaw = target.yaw;
  target_.pitch = Clamp(target.pitch, kPitchMin, kPitchMax);
}

void Gimbal::update() {
  if (!yaw_motor_ || !pitch_motor_ || !yaw_pid_ || !pitch_pid_) {
    stop();
    return;
  }

  const GimbalMotion current = feedback();
  const float yaw_error = WrapPi(target_.yaw - current.yaw);
  const float yaw_output = yaw_pid_->calculate(yaw_error, 0.0f);
  float pitch_output = pitch_pid_->calculate(target_.pitch, current.pitch);

  // 已越过软限位时，只允许朝允许区间内部发力。
  if ((current.pitch <= kPitchMin && pitch_output < 0.0f) ||
      (current.pitch >= kPitchMax && pitch_output > 0.0f)) {
    pitch_output = 0.0f;
  }
  yaw_motor_->setInput(yaw_output);
  pitch_motor_->setInput(pitch_output);
}

void Gimbal::stop() {
  if (yaw_motor_) yaw_motor_->setInput(0.0f);
  if (pitch_motor_) pitch_motor_->setInput(0.0f);
  if (yaw_pid_) yaw_pid_->reset();
  if (pitch_pid_) pitch_pid_->reset();
}

}  // namespace robot

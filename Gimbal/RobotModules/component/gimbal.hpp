/**
 *******************************************************************************
 * @file      : gimbal.hpp
 * @brief     : DM-J4310 yaw/pitch 关节空间位置控制
 *******************************************************************************
 */
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

#include "module_state.hpp"
#include "motor.hpp"
#include "pid.hpp"

namespace robot {

class Gimbal {
 public:
  // 初次上机采用保守限位；机械实测后只允许在这里统一调整。
  static constexpr float kPitchMin = -0.30f;
  static constexpr float kPitchMax = 0.30f;

  void registerYaw(hello_world::motor::Motor *motor,
                   hello_world::pid::Pid *pid);
  void registerPitch(hello_world::motor::Motor *motor,
                     hello_world::pid::Pid *pid);

  void resetTarget();
  void setTarget(const GimbalMotion &target);
  void update();
  void stop();

  const GimbalMotion &target() const { return target_; }
  GimbalMotion feedback() const;

 private:
  hello_world::motor::Motor *yaw_motor_ = nullptr;
  hello_world::motor::Motor *pitch_motor_ = nullptr;
  hello_world::pid::Pid *yaw_pid_ = nullptr;
  hello_world::pid::Pid *pitch_pid_ = nullptr;
  GimbalMotion target_{};
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */

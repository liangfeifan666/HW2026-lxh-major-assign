/**
 *******************************************************************************
 * @file      : chassis_ik_solver.cpp
 * @brief     : 标准四轮麦轮逆解；超限等比缩放以保持运动方向
 *******************************************************************************
 */
#include "chassis_ik_solver.hpp"

#include <algorithm>
#include <cmath>

namespace hello_world {
namespace chassis_ik_solver {

ChassisIkSolver::ChassisIkSolver(float wheel_radius, float half_length,
                                 float half_width, float max_wheel_speed)
    : wheel_radius_(wheel_radius),
      rotation_radius_(half_length + half_width),
      max_wheel_speed_(max_wheel_speed) {}

ChassisIkSolver::WheelSpeeds ChassisIkSolver::solve(
    const robot::ChassisMotion &motion, float gimbal_yaw) const {
  WheelSpeeds wheel_speeds{};
  if (!std::isfinite(motion.vx) || !std::isfinite(motion.vy) ||
      !std::isfinite(motion.w) || !std::isfinite(gimbal_yaw) ||
      wheel_radius_ <= 0.0f) {
    return wheel_speeds;
  }

  const float delta = robot::WrapPi(gimbal_yaw);
  const float cos_delta = std::cos(delta);
  const float sin_delta = std::sin(delta);
  const float vx = motion.vx * cos_delta + motion.vy * sin_delta;
  const float vy = -motion.vx * sin_delta + motion.vy * cos_delta;
  const float rotation = rotation_radius_ * motion.w;

  // x 前、y 左、z 上；正 w 为绕 z 轴逆时针。
  wheel_speeds[kLeftFront] = (vx - vy - rotation) / wheel_radius_;
  wheel_speeds[kLeftBack] = (vx + vy - rotation) / wheel_radius_;
  wheel_speeds[kRightBack] = (vx - vy + rotation) / wheel_radius_;
  wheel_speeds[kRightFront] = (vx + vy + rotation) / wheel_radius_;

  float max_abs = 0.0f;
  for (float speed : wheel_speeds) max_abs = std::max(max_abs, std::fabs(speed));
  if (max_abs > max_wheel_speed_) {
    const float scale = max_wheel_speed_ / max_abs;
    for (float &speed : wheel_speeds) speed *= scale;
  }
  return wheel_speeds;
}

}  // namespace chassis_ik_solver
}  // namespace hello_world

/**
 *******************************************************************************
 * @file      : chassis_ik_solver.hpp
 * @brief     : 云台系速度到底盘四麦轮轮速的逆运动学
 *******************************************************************************
 */
#ifndef CHASSIS_IK_SOLVER_HPP_
#define CHASSIS_IK_SOLVER_HPP_

#include <array>
#include <cstddef>

#include "module_state.hpp"

namespace hello_world {
namespace chassis_ik_solver {

class ChassisIkSolver {
 public:
  static constexpr std::size_t kWheelCount = 4;
  using WheelSpeeds = std::array<float, kWheelCount>;

  // 数组顺序固定为 LF、LB、RB、RF，与电机实例顺序一致。
  enum Wheel : std::size_t { kLeftFront = 0, kLeftBack, kRightBack, kRightFront };

  ChassisIkSolver(float wheel_radius = 0.07786f,
                  float half_length = 0.15338f,
                  float half_width = 0.15338f,
                  float max_wheel_speed = 40.0f);

  // motion 定义在云台系；gimbal_yaw = 云台相对底盘的关节角，单位 rad。
  WheelSpeeds solve(const robot::ChassisMotion &motion, float gimbal_yaw) const;

 private:
  float wheel_radius_;
  float rotation_radius_;
  float max_wheel_speed_;
};

}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* CHASSIS_IK_SOLVER_HPP_ */

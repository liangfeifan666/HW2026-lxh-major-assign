/**
 *******************************************************************************
 * @file      : chassis.hpp
 * @brief     : 底盘运动模块，保存本周期指令与四轮逆解结果
 *******************************************************************************
 */
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

#include "chassis_ik_solver.hpp"
#include "module_state.hpp"

namespace robot {

class Chassis {
 public:
  using WheelSpeeds = hello_world::chassis_ik_solver::ChassisIkSolver::WheelSpeeds;

  void registerIkSolver(
      hello_world::chassis_ik_solver::ChassisIkSolver *solver) {
    solver_ = solver;
  }

  void update(const ChassisMotion &motion, float gimbal_yaw);
  void stop();

  const WheelSpeeds &wheel_speeds() const { return wheel_speeds_; }
  const ChassisMotion &motion() const { return motion_; }

 private:
  hello_world::chassis_ik_solver::ChassisIkSolver *solver_ = nullptr;
  ChassisMotion motion_{};
  WheelSpeeds wheel_speeds_{};
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */

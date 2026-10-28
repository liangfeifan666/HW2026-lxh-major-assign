/**
 *******************************************************************************
 * @file      : chassis.cpp
 * @brief     : 调用共享逆解器；未注册或异常输入时保持全零
 *******************************************************************************
 */
#include "chassis.hpp"

#include <cmath>

namespace robot {

void Chassis::update(const ChassisMotion &motion, float gimbal_yaw) {
  if (solver_ == nullptr || !std::isfinite(motion.vx) ||
      !std::isfinite(motion.vy) || !std::isfinite(motion.w)) {
    stop();
    return;
  }
  motion_ = motion;
  wheel_speeds_ = solver_->solve(motion_, gimbal_yaw);
}

void Chassis::stop() {
  motion_ = {};
  wheel_speeds_.fill(0.0f);
}

}  // namespace robot

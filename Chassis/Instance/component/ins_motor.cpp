/**
 *******************************************************************************
 * @file      : ins_motor.cpp
 * @brief     : 四轮 M3508 与独立反馈离线检测
 *******************************************************************************
 */
#include "ins_motor.hpp"

#include "motor_M3508.hpp"
#include "offline_checker.hpp"

namespace {
constexpr uint8_t kWheelIds[4] = {1, 2, 3, 4};  // LF、LB、RB、RF
}

hello_world::motor::Motor *GetMotorWheel(int idx) {
  using namespace hello_world::motor;
  static const OptionalParams opt = [] {
    OptionalParams value;
    value.input_type = InputType::kRaw;
    value.dir = kDirRev;  // 任务书 6.2：四个轮电机均反向
    value.max_raw_input_lim = 6000.0f;
    return value;
  }();
  static M3508 motors[] = {
      M3508(kWheelIds[0], opt), M3508(kWheelIds[1], opt),
      M3508(kWheelIds[2], opt), M3508(kWheelIds[3], opt)};
  return (idx >= 0 && idx < 4) ? &motors[idx] : nullptr;
}

hello_world::OfflineChecker *GetMotorWheelOfflineChecker(int idx) {
  static hello_world::OfflineChecker checkers[4];
  return (idx >= 0 && idx < 4) ? &checkers[idx] : nullptr;
}

bool DecodeMotorWheelFeedback(uint32_t std_id, const uint8_t data[8]) {
  for (int i = 0; i < 4; ++i) {
    if (std_id != hello_world::motor::M3508::GetRxId(kWheelIds[i])) continue;
    if (!GetMotorWheel(i)->decode(data)) return true;
    GetMotorWheelOfflineChecker(i)->update();
    return true;
  }
  return false;
}

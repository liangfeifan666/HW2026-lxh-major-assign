/**
 *******************************************************************************
 * @file      : ins_motor.cpp
 * @brief     : yaw/pitch DM-J4310 参数构造与反馈离线检测
 *******************************************************************************
 */
#include "ins_motor.hpp"

#include "motor_DaMiao.hpp"
#include "offline_checker.hpp"

namespace {
constexpr uint8_t kYawId = 1;
constexpr uint8_t kPitchId = 2;

hello_world::motor::OptionalParams MakeOptions(hello_world::motor::Dir dir,
                                               float angle_offset) {
  using namespace hello_world::motor;
  OptionalParams opt;
  opt.input_type = InputType::kTorq;  // DM-J4310 禁止沿用默认 kRaw
  opt.angle_range = AngleRange::kNegInfToPosInf;
  opt.dir = dir;
  opt.angle_offset = angle_offset;
  return opt;
}
}  // namespace

hello_world::motor::Motor *GetMotorYaw() {
  static const auto opt = MakeOptions(hello_world::motor::kDirFwd, -1.36920f);
  static hello_world::motor::DM_J4310 motor(kYawId, opt, false, false);
  return &motor;
}

hello_world::motor::Motor *GetMotorPitch() {
  static const auto opt = MakeOptions(hello_world::motor::kDirRev, 0.02480f);
  static hello_world::motor::DM_J4310 motor(kPitchId, opt, false, false);
  return &motor;
}

hello_world::OfflineChecker *GetMotorYawOfflineChecker() {
  static hello_world::OfflineChecker checker;
  return &checker;
}

hello_world::OfflineChecker *GetMotorPitchOfflineChecker() {
  static hello_world::OfflineChecker checker;
  return &checker;
}

bool DecodeGimbalMotorFeedback(uint32_t std_id, const uint8_t data[8]) {
  hello_world::motor::Motor *motor = nullptr;
  hello_world::OfflineChecker *checker = nullptr;
  if (std_id == hello_world::motor::DM_J4310::GetRxId(kYawId)) {
    motor = GetMotorYaw();
    checker = GetMotorYawOfflineChecker();
  } else if (std_id == hello_world::motor::DM_J4310::GetRxId(kPitchId)) {
    motor = GetMotorPitch();
    checker = GetMotorPitchOfflineChecker();
  } else {
    return false;
  }
  if (motor->decode(data)) checker->update();
  return true;
}

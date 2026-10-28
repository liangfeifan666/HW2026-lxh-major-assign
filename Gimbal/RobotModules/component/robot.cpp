/**
 *******************************************************************************
 * @file      : robot.cpp
 * @brief     : 急停优先；右拨杆控制分离/跟随，左拨杆上档进入限速小陀螺
 *******************************************************************************
 */
#include "robot.hpp"

#include <algorithm>
#include <cmath>

#include "motor_DaMiao.hpp"

namespace {
using hello_world::remote_control::kSwitchStateDown;
using hello_world::remote_control::kSwitchStateMid;
using hello_world::remote_control::kSwitchStateUp;

float Deadzone(float value, float threshold = 0.03f) {
  value = robot::Clamp(value, -1.0f, 1.0f);
  return std::fabs(value) < threshold ? 0.0f : value;
}
}  // namespace

namespace robot {

void Robot::registerRemote(hello_world::remote_control::DT7 *remote,
                           hello_world::OfflineChecker *checker) {
  remote_ = remote;
  remote_checker_ = checker;
}

void Robot::registerMotors(hello_world::motor::Motor *yaw,
                           hello_world::motor::Motor *pitch,
                           hello_world::OfflineChecker *yaw_checker,
                           hello_world::OfflineChecker *pitch_checker) {
  yaw_motor_ = yaw;
  pitch_motor_ = pitch;
  yaw_checker_ = yaw_checker;
  pitch_checker_ = pitch_checker;
}

bool Robot::hardwareReady() const {
  if (!gimbal_ || !imu_ || !comm_ || !remote_ || !remote_checker_ ||
      !yaw_motor_ || !pitch_motor_ || !yaw_checker_ || !pitch_checker_ ||
      !imu_->isReady() || remote_checker_->isOffline() ||
      yaw_checker_->isOffline() || pitch_checker_->isOffline() ||
      comm_->isOffline()) {
    return false;
  }
  const auto *yaw = static_cast<hello_world::motor::DaMiao *>(yaw_motor_);
  const auto *pitch = static_cast<hello_world::motor::DaMiao *>(pitch_motor_);
  return yaw->is_enabled() && pitch->is_enabled() &&
         yaw->status_code() == hello_world::motor::DaMiao::kMotorEnabled &&
         pitch->status_code() == hello_world::motor::DaMiao::kMotorEnabled;
}

bool Robot::validModeSwitch() const {
  if (!remote_) return false;
  const auto right = right_switch_;
  const auto left = left_switch_;
  return (right == kSwitchStateMid || right == kSwitchStateUp) &&
         (left == kSwitchStateMid || left == kSwitchStateUp);
}

ChassisMode Robot::chassisMode() const {
  if (right_switch_ == kSwitchStateMid) return ChassisMode::Separate;
  return left_switch_ == kSwitchStateUp ? ChassisMode::Spin
                                                  : ChassisMode::Follow;
}

bool Robot::sticksCentered() const {
  return std::fabs(rc_lv_) < 0.05f &&
         std::fabs(rc_lh_) < 0.05f &&
         std::fabs(rc_rv_) < 0.05f &&
         std::fabs(rc_rh_) < 0.05f &&
         std::fabs(rc_wheel_) < 0.05f;
}

bool Robot::outputAllowed() const {
  return life_state_ == LifeState::Working && hardwareReady() &&
         unlock_armed_ && validModeSwitch();
}

void Robot::updateState() {
  const bool remote_online = remote_ && remote_checker_ &&
                             !remote_checker_->isOffline();
  const bool estop = remote_online &&
                     right_switch_ == kSwitchStateDown;
  const bool failed = !hardwareReady() || estop ||
                      (!estop && (!unlock_armed_ || !validModeSwitch()));
  if (!remote_online || (life_state_ == LifeState::Working && failed)) {
    // 遥控断线或运行中发生故障后，必须重新经历一次明确锁定档。
    unlock_armed_ = false;
  }
  if (estop) unlock_armed_ = true;

  if (failed) {
    life_state_ = LifeState::Dead;
    resurrection_ticks_ = 0;
    return;
  }

  if (life_state_ == LifeState::Dead) {
    life_state_ = LifeState::Resurrection;
    resurrection_ticks_ = 0;
    gimbal_->resetTarget();
  } else if (life_state_ == LifeState::Resurrection) {
    resurrection_ticks_ = sticksCentered()
                              ? static_cast<uint16_t>(resurrection_ticks_ + 1)
                              : 0;
    if (resurrection_ticks_ >= kResurrectionTicks) {
      life_state_ = LifeState::Working;
    }
  }
}

void Robot::update() {
  if (remote_) {
    // UART Rx 可抢占控制中断，所有拨杆和模拟量必须使用同一帧快照。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    left_switch_ = remote_->rc_l_switch();
    right_switch_ = remote_->rc_r_switch();
    rc_lv_ = remote_->rc_lv();
    rc_lh_ = remote_->rc_lh();
    rc_rv_ = remote_->rc_rv();
    rc_rh_ = remote_->rc_rh();
    rc_wheel_ = remote_->rc_wheel();
    __set_PRIMASK(primask);
  }
  if (imu_) imu_->update();
  updateState();
  if (life_state_ == LifeState::Resurrection && gimbal_) {
    // 等待期间持续跟随实际位置，避免解锁瞬间拉回 300ms 前的旧目标。
    gimbal_->resetTarget();
  }
  chassis_mode_ = (remote_ && validModeSwitch()) ? chassisMode()
                                                 : ChassisMode::Separate;
  if (!outputAllowed()) {
    stopAll();
    updateComm(false, {});
    return;
  }
  runWorking();
}

void Robot::runWorking() {
  if (!std::isfinite(rc_lv_) || !std::isfinite(rc_lh_) ||
      !std::isfinite(rc_rv_) || !std::isfinite(rc_rh_) ||
      !std::isfinite(rc_wheel_)) {
    life_state_ = LifeState::Dead;
    stopAll();
    updateComm(false, {});
    return;
  }

  GimbalMotion target = gimbal_->target();
  target.yaw -= Deadzone(rc_rh_) * 1.5f * 0.001f;
  target.pitch += Deadzone(rc_rv_) * 0.8f * 0.001f;
  gimbal_->setTarget(target);
  gimbal_->update();

  ChassisMotion command{
      Deadzone(rc_lv_) * 1.2f,
      Deadzone(rc_lh_) * 1.2f,
      0.0f,
  };
  if (chassis_mode_ != ChassisMode::Follow) {
    command.w = Deadzone(rc_wheel_) * 5.0f;
  }
  updateComm(true, command);
}

void Robot::stopAll() {
  if (gimbal_) gimbal_->stop();
}

void Robot::updateComm(bool remote_working, const ChassisMotion &command) {
  if (!comm_) return;
  auto &tx = comm_->gimbal_data();
  tx.life = life_state_;
  tx.mode = GimbalMode::JointSpace;
  tx.chassis_mode = chassis_mode_;
  tx.remote_working = remote_working;
  tx.chassis_command = command;
  const GimbalMotion current = gimbal_ ? gimbal_->feedback() : GimbalMotion{};
  tx.yaw = current.yaw;
  tx.pitch = current.pitch;
}

}  // namespace robot

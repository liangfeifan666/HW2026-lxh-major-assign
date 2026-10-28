/**
 *******************************************************************************
 * @file      : robot.cpp
 * @brief     : 底盘仅在云台板确认遥控有效后运行；任何失效持续发零
 *******************************************************************************
 */
#include "robot.hpp"

#include <cmath>

#include "comm_task.hpp"

namespace robot {

void Robot::registerWheel(int idx, hello_world::motor::Motor *motor,
                          hello_world::pid::Pid *pid,
                          hello_world::OfflineChecker *checker) {
  if (idx < 0 || idx >= static_cast<int>(kWheelCount)) return;
  motors_[idx] = motor;
  pids_[idx] = pid;
  checkers_[idx] = checker;
}

bool Robot::hardwareReady() const {
  if (!chassis_ || !imu_ || !comm_ || !follow_pid_ || !imu_->isReady() ||
      !CommMotorCanHealthy()) {
    return false;
  }
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    if (!motors_[i] || !pids_[i] || !checkers_[i] ||
        checkers_[i]->isOffline()) {
      return false;
    }
  }
  return true;
}

bool Robot::outputAllowed() const {
  if (life_state_ != LifeState::Working || !hardwareReady() ||
      comm_->isOffline()) {
    return false;
  }
  const auto &gimbal = gimbal_data_;
  return gimbal.life == LifeState::Working && gimbal.remote_working;
}

void Robot::updateState() {
  const bool peer_ready = comm_ && !comm_->isOffline() &&
                          gimbal_data_.life == LifeState::Working &&
                          gimbal_data_.remote_working;
  if (!hardwareReady() || !peer_ready) {
    life_state_ = LifeState::Dead;
    resurrection_ticks_ = 0;
    return;
  }

  if (life_state_ == LifeState::Dead) {
    life_state_ = LifeState::Resurrection;
    resurrection_ticks_ = 0;
  } else if (life_state_ == LifeState::Resurrection) {
    if (++resurrection_ticks_ >= kResurrectionTicks) {
      life_state_ = LifeState::Working;
    }
  }
}

void Robot::update() {
  if (comm_) {
    // CAN Rx 优先级更高，短临界区保证安全位、指令和角度来自同一帧。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    gimbal_data_ = comm_->gimbal_data();
    __set_PRIMASK(primask);
  }
  // 对端急停/离线是最高优先级，先于一切非必要计算。
  if (!comm_ || comm_->isOffline() ||
      gimbal_data_.life != LifeState::Working ||
      !gimbal_data_.remote_working) {
    life_state_ = LifeState::Dead;
    resurrection_ticks_ = 0;
    stopAll();
    updateComm(false, 0.0f);
    return;
  }
  updateState();

  if (!outputAllowed()) {
    stopAll();
    updateComm(false, 0.0f);
    return;
  }
  runWorking();
}

void Robot::runWorking() {
  const auto &gimbal = gimbal_data_;
  ChassisMotion command = gimbal.chassis_command;
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
      !std::isfinite(command.w) || !std::isfinite(gimbal.yaw)) {
    life_state_ = LifeState::Dead;
    stopAll();
    updateComm(false, 0.0f);
    return;
  }

  command.vx = Clamp(command.vx, -1.2f, 1.2f);
  command.vy = Clamp(command.vy, -1.2f, 1.2f);
  const float delta = WrapPi(gimbal.yaw);
  if (gimbal.chassis_mode == ChassisMode::Follow) {
    // 误差跨 ±π 时用最短弧变化量，避免把支路跳变放大成微分脉冲。
    const float delta_derivative = has_previous_delta_
                                       ? WrapPi(delta - previous_delta_) / 0.001f
                                       : 0.0f;
    previous_delta_ = delta;
    has_previous_delta_ = true;
    command.w = follow_pid_->calculate(delta, 0.0f, delta_derivative);
  } else {
    follow_pid_->reset();
    has_previous_delta_ = false;
    command.w = Clamp(command.w, -5.0f, 5.0f);
  }
  command.w = Clamp(command.w, -5.0f, 5.0f);

  // 零输入必须真的是零电流，不能让速度环对残余轮速继续制动。
  if (std::fabs(command.vx) < 1e-6f &&
      std::fabs(command.vy) < 1e-6f &&
      std::fabs(command.w) < 1e-6f) {
    stopAll();
    updateComm(false, 0.0f);
    return;
  }

  chassis_->update(command, delta);
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    const float output = pids_[i]->calculate(chassis_->wheel_speeds()[i],
                                             motors_[i]->vel());
    motors_[i]->setInput(output);
  }
  updateComm(true, command.w);
}

void Robot::stopAll() {
  if (chassis_) chassis_->stop();
  if (follow_pid_) follow_pid_->reset();
  has_previous_delta_ = false;
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    if (motors_[i]) motors_[i]->setInput(0.0f);
    if (pids_[i]) pids_[i]->reset();
  }
}

void Robot::updateComm(bool output_enabled, float actual_w) {
  if (!comm_) return;
  auto &tx = comm_->chassis_data();
  tx.life = life_state_;
  tx.mode = comm_->isOffline() ? ChassisMode::Separate
                               : gimbal_data_.chassis_mode;
  tx.output_enabled = output_enabled;
  tx.yaw = imu_ ? imu_->yaw() : 0.0f;
  tx.w = actual_w;
}

}  // namespace robot

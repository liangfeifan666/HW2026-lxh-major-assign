/**
 *******************************************************************************
 * @file      : robot.hpp
 * @brief     : 底盘三态状态机与统一安全输出出口
 *******************************************************************************
 */
#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

#include <array>
#include <cstdint>

#include "chassis.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "offline_checker.hpp"
#include "pid.hpp"

namespace robot {

class Robot {
 public:
  void registerChassis(Chassis *chassis) { chassis_ = chassis; }
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerComm(GimbalChassisComm *comm) { comm_ = comm; }
  void registerFollowPid(hello_world::pid::Pid *pid) { follow_pid_ = pid; }
  void registerWheel(int idx, hello_world::motor::Motor *motor,
                     hello_world::pid::Pid *pid,
                     hello_world::OfflineChecker *checker);

  void update();
  bool outputAllowed() const;
  LifeState life_state() const { return life_state_; }

 private:
  static constexpr std::size_t kWheelCount = 4;
  static constexpr uint16_t kResurrectionTicks = 300;

  bool hardwareReady() const;
  void updateState();
  void runWorking();
  void stopAll();
  void updateComm(bool output_enabled, float actual_w);

  Chassis *chassis_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;
  hello_world::pid::Pid *follow_pid_ = nullptr;
  std::array<hello_world::motor::Motor *, kWheelCount> motors_{};
  std::array<hello_world::pid::Pid *, kWheelCount> pids_{};
  std::array<hello_world::OfflineChecker *, kWheelCount> checkers_{};
  LifeState life_state_ = LifeState::Dead;
  GimbalChassisComm::GimbalData gimbal_data_{};
  float previous_delta_ = 0.0f;
  bool has_previous_delta_ = false;
  uint16_t resurrection_ticks_ = 0;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */

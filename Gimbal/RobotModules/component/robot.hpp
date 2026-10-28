/**
 *******************************************************************************
 * @file      : robot.hpp
 * @brief     : 云台三态状态机、遥控解锁与整车指令入口
 *******************************************************************************
 */
#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

#include <cstdint>

#include "DT7.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "offline_checker.hpp"

namespace robot {

class Robot {
 public:
  void registerGimbal(Gimbal *gimbal) { gimbal_ = gimbal; }
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerComm(GimbalChassisComm *comm) { comm_ = comm; }
  void registerRemote(hello_world::remote_control::DT7 *remote,
                      hello_world::OfflineChecker *checker);
  void registerMotors(hello_world::motor::Motor *yaw,
                      hello_world::motor::Motor *pitch,
                      hello_world::OfflineChecker *yaw_checker,
                      hello_world::OfflineChecker *pitch_checker);

  void update();
  bool outputAllowed() const;
  LifeState life_state() const { return life_state_; }

 private:
  static constexpr uint16_t kResurrectionTicks = 300;

  bool hardwareReady() const;
  bool validModeSwitch() const;
  bool sticksCentered() const;
  ChassisMode chassisMode() const;
  void updateState();
  void runWorking();
  void stopAll();
  void updateComm(bool remote_working, const ChassisMotion &command);

  Gimbal *gimbal_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;
  hello_world::remote_control::DT7 *remote_ = nullptr;
  hello_world::OfflineChecker *remote_checker_ = nullptr;
  hello_world::motor::Motor *yaw_motor_ = nullptr;
  hello_world::motor::Motor *pitch_motor_ = nullptr;
  hello_world::OfflineChecker *yaw_checker_ = nullptr;
  hello_world::OfflineChecker *pitch_checker_ = nullptr;
  LifeState life_state_ = LifeState::Dead;
  ChassisMode chassis_mode_ = ChassisMode::Separate;
  GimbalChassisComm::ChassisData chassis_data_{};
  bool peer_was_working_ = false;
  hello_world::remote_control::SwitchState left_switch_ =
      hello_world::remote_control::kSwitchStateDown;
  hello_world::remote_control::SwitchState right_switch_ =
      hello_world::remote_control::kSwitchStateDown;
  float rc_lv_ = 0.0f;
  float rc_lh_ = 0.0f;
  float rc_rv_ = 0.0f;
  float rc_rh_ = 0.0f;
  float rc_wheel_ = 0.0f;
  uint16_t resurrection_ticks_ = 0;
  uint32_t last_enable_retry_ms_ = 0;
  bool unlock_armed_ = false;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */

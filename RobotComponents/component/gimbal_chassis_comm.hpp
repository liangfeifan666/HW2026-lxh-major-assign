/**
 *******************************************************************************
 * @file      : gimbal_chassis_comm.hpp
 * @brief     : 底盘板与云台板之间的 8 字节 CAN 心跳/状态协议
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

#include <cstdint>

#include "module_state.hpp"
#include "tick.hpp"

namespace robot {

class GimbalChassisComm {
 public:
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };

  struct ChassisData {
    LifeState life = LifeState::Dead;
    ChassisMode mode = ChassisMode::Separate;
    bool output_enabled = false;
    float yaw = 0.0f;  // 底盘 IMU 相对开机朝向，rad
    float w = 0.0f;    // 实际下发的底盘角速度，rad/s
  };

  struct GimbalData {
    LifeState life = LifeState::Dead;
    GimbalMode mode = GimbalMode::JointSpace;
    ChassisMode chassis_mode = ChassisMode::Separate;
    bool remote_working = false;  // 遥控有效、已解锁且本板允许输出
    float yaw = 0.0f;             // 云台相对底盘 yaw 关节角，rad
    float pitch = 0.0f;           // 云台 pitch 关节角，rad
    ChassisMotion chassis_command;  // 已由遥控器生成的底盘指令
  };

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  void encode(uint8_t tx_data[8]);
  void decode(const uint8_t rx_data[8]);
  bool isOffline() const;

  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }
  uint32_t tx_id() const { return tx_id_; }
  uint32_t rx_id() const { return rx_id_; }

 private:
  static constexpr uint32_t kOfflineTimeoutMs = 100;

  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;
  bool received_ = false;  // 从未收到对端帧时必须判离线
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
};

}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */

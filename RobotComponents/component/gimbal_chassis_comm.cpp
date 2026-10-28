/**
 *******************************************************************************
 * @file      : gimbal_chassis_comm.cpp
 * @brief     : 双板协议编解码；定点比例必须在收发两端保持一致
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#include "gimbal_chassis_comm.hpp"

#include <cmath>

namespace {
constexpr uint8_t kProtocolVersion = 0xA0;

int16_t EncodeI16(float value, float scale, float limit) {
  if (!std::isfinite(value)) return 0;
  value = robot::Clamp(value, -limit, limit);
  return static_cast<int16_t>(value * scale);
}

int8_t EncodeI8(float value, float scale, float limit) {
  if (!std::isfinite(value)) return 0;
  value = robot::Clamp(value, -limit, limit);
  return static_cast<int8_t>(value * scale);
}

void PutI16(uint8_t *data, int index, int16_t value) {
  const uint16_t raw = static_cast<uint16_t>(value);
  data[index] = static_cast<uint8_t>(raw >> 8);
  data[index + 1] = static_cast<uint8_t>(raw & 0xFF);
}

int16_t GetI16(const uint8_t *data, int index) {
  const uint16_t raw = (static_cast<uint16_t>(data[index]) << 8) | data[index + 1];
  return static_cast<int16_t>(raw);
}

bool ValidCommon(const uint8_t data[8]) {
  const uint8_t life = data[0] & 0x0F;
  const uint8_t mode = data[1] & 0x0F;
  const bool enabled = (data[0] & 0x80) != 0;
  return (data[0] & 0x70) == 0 && life <= 2 && mode <= 2 &&
         (!enabled || life == 2);  // 放行位只能与 Working 同时出现。
}

bool ValidI8(const uint8_t data[8], int index, int limit) {
  const int value = static_cast<int8_t>(data[index]);
  return value >= -limit && value <= limit;
}

bool ValidI16(const uint8_t data[8], int index, int limit) {
  const int value = GetI16(data, index);
  return value >= -limit && value <= limit;
}

robot::LifeState DecodeLife(uint8_t value) {
  const auto life = static_cast<robot::LifeState>(value & 0x0F);
  return life <= robot::LifeState::Working ? life : robot::LifeState::Dead;
}

robot::ChassisMode DecodeMode(uint8_t value) {
  const auto mode = static_cast<robot::ChassisMode>(value & 0x0F);
  return mode <= robot::ChassisMode::Spin ? mode : robot::ChassisMode::Separate;
}
}  // namespace

namespace robot {

GimbalChassisComm::GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id)
    : code_part_(code_part), tx_id_(tx_id), rx_id_(rx_id),
      last_rx_tick_(hello_world::tick::GetTickMs()) {}

void GimbalChassisComm::encode(uint8_t tx_data[8]) {
  for (int i = 0; i < 8; ++i) tx_data[i] = 0;

  if (code_part_ == CodePart::Chassis) {
    const auto &data = chassis_data_;
    const auto life = data.life <= LifeState::Working ? data.life : LifeState::Dead;
    const auto mode = data.mode <= ChassisMode::Spin ? data.mode : ChassisMode::Separate;
    tx_data[0] = static_cast<uint8_t>(life) | (data.output_enabled ? 0x80 : 0);
    tx_data[1] = kProtocolVersion | static_cast<uint8_t>(mode);
    tx_data[3] = static_cast<uint8_t>(EncodeI8(data.w, 20.0f, 5.0f));
    PutI16(tx_data, 4, EncodeI16(WrapPi(data.yaw), 10000.0f, 3.14f));
  } else {
    const auto &data = gimbal_data_;
    const auto life = data.life <= LifeState::Working ? data.life : LifeState::Dead;
    const auto mode = data.chassis_mode <= ChassisMode::Spin
                          ? data.chassis_mode : ChassisMode::Separate;
    tx_data[0] = static_cast<uint8_t>(life) | (data.remote_working ? 0x80 : 0);
    tx_data[1] = kProtocolVersion | static_cast<uint8_t>(mode);
    tx_data[2] = static_cast<uint8_t>(EncodeI8(data.chassis_command.vx, 100.0f, 1.2f));
    tx_data[3] = static_cast<uint8_t>(EncodeI8(data.chassis_command.vy, 100.0f, 1.2f));
    tx_data[4] = static_cast<uint8_t>(EncodeI8(data.chassis_command.w, 20.0f, 5.0f));
    tx_data[5] = static_cast<uint8_t>(EncodeI8(data.pitch, 100.0f, 1.2f));
    PutI16(tx_data, 6, EncodeI16(WrapPi(data.yaw), 10000.0f, 3.14f));
  }
}

void GimbalChassisComm::decode(const uint8_t rx_data[8]) {
  // 版本、枚举、保留位或定点范围不符的帧直接丢弃，不刷新在线时间。
  if ((rx_data[1] & 0xF0) != kProtocolVersion || !ValidCommon(rx_data)) return;

  if (code_part_ == CodePart::Chassis) {
    if (!ValidI8(rx_data, 2, 120) || !ValidI8(rx_data, 3, 120) ||
        !ValidI8(rx_data, 4, 100) || !ValidI8(rx_data, 5, 120) ||
        !ValidI16(rx_data, 6, 31400)) return;
    auto &data = gimbal_data_;
    data.life = DecodeLife(rx_data[0]);
    data.remote_working = (rx_data[0] & 0x80) != 0;
    data.chassis_mode = DecodeMode(rx_data[1]);
    data.chassis_command.vx = static_cast<int8_t>(rx_data[2]) / 100.0f;
    data.chassis_command.vy = static_cast<int8_t>(rx_data[3]) / 100.0f;
    data.chassis_command.w = static_cast<int8_t>(rx_data[4]) / 20.0f;
    data.pitch = static_cast<int8_t>(rx_data[5]) / 100.0f;
    data.yaw = static_cast<float>(GetI16(rx_data, 6)) / 10000.0f;
  } else {
    if (rx_data[2] != 0 || rx_data[6] != 0 || rx_data[7] != 0 ||
        !ValidI8(rx_data, 3, 100) || !ValidI16(rx_data, 4, 31400)) return;
    auto &data = chassis_data_;
    data.life = DecodeLife(rx_data[0]);
    data.output_enabled = (rx_data[0] & 0x80) != 0;
    data.mode = DecodeMode(rx_data[1]);
    data.w = static_cast<int8_t>(rx_data[3]) / 20.0f;
    data.yaw = static_cast<float>(GetI16(rx_data, 4)) / 10000.0f;
  }

  last_rx_tick_ = hello_world::tick::GetTickMs();
  received_ = true;
}

bool GimbalChassisComm::isOffline() const {
  if (!received_) return true;
  return hello_world::tick::GetTickMs() - last_rx_tick_ > kOfflineTimeoutMs;
}

}  // namespace robot

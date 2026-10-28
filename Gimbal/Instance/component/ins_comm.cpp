/**
 *******************************************************************************
 * @file      : ins_comm.cpp
 * @brief     : DT7 单例与遥控帧新鲜度管理
 *******************************************************************************
 */
#include "ins_comm.hpp"

#include "offline_checker.hpp"

hello_world::remote_control::DT7 *GetRemoteControl() {
  static hello_world::remote_control::DT7 remote;
  return &remote;
}

hello_world::OfflineChecker *GetRemoteControlOfflineChecker() {
  static hello_world::OfflineChecker checker;
  return &checker;
}

namespace {
bool ValidChannel(uint32_t value) {
  // DT7 常见有效脉宽映射范围；异常值不得刷新遥控在线状态。
  return value >= 364 && value <= 1684;
}

bool ValidRemoteFrame(const uint8_t data[18]) {
  const uint32_t rh = (data[0] | (data[1] << 8)) & 0x07FF;
  const uint32_t rv = ((data[1] >> 3) | (data[2] << 5)) & 0x07FF;
  const uint32_t lh = ((data[2] >> 6) | (data[3] << 2) |
                       (data[4] << 10)) & 0x07FF;
  const uint32_t lv = ((data[4] >> 1) | (data[5] << 7)) & 0x07FF;
  const uint32_t wheel = data[16] | (data[17] << 8);
  const uint8_t left = (data[5] >> 6) & 0x03;
  const uint8_t right = (data[5] >> 4) & 0x03;
  const bool switches_valid = left >= 1 && left <= 3 &&
                              right >= 1 && right <= 3;
  return switches_valid && ValidChannel(rh) && ValidChannel(rv) &&
         ValidChannel(lh) && ValidChannel(lv) && ValidChannel(wheel);
}
}  // namespace

bool DecodeRemoteControlFrame(const uint8_t data[18]) {
  if (!ValidRemoteFrame(data)) return false;
  GetRemoteControl()->decode(data);
  GetRemoteControlOfflineChecker()->update();
  return true;
}

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

void DecodeRemoteControlFrame(const uint8_t data[18]) {
  GetRemoteControl()->decode(data);
  GetRemoteControlOfflineChecker()->update();
}

/**
 *******************************************************************************
 * @file      : ins_pid.cpp
 * @brief     : 云台两轴位置环参数集中配置
 *******************************************************************************
 */
#include "ins_pid.hpp"

#include "pid.hpp"

hello_world::pid::Pid *GetPidMotorYaw() {
  // 输出为达妙 MIT 力矩 N·m；首次调试限制在较小范围。
  static hello_world::pid::Pid pid(2.5f, 0.0f, 0.02f, 0.2f, 2.0f);
  return &pid;
}

hello_world::pid::Pid *GetPidMotorPitch() {
  static hello_world::pid::Pid pid(3.0f, 1.0f, 0.02f, 0.3f, 2.0f);
  return &pid;
}

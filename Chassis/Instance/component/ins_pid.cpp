/**
 *******************************************************************************
 * @file      : ins_pid.cpp
 * @brief     : 底盘四轮速度环与跟随角环参数集中配置
 *******************************************************************************
 */
#include "ins_pid.hpp"

#include "pid.hpp"

hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  // 初始增益仅用于车轮离地、小指令调试；输出限幅先于满电流验证。
  static hello_world::pid::Pid pids[] = {
      {800.0f, 100.0f, 0.0f, 4.0f, 6000.0f},
      {800.0f, 100.0f, 0.0f, 4.0f, 6000.0f},
      {800.0f, 100.0f, 0.0f, 4.0f, 6000.0f},
      {800.0f, 100.0f, 0.0f, 4.0f, 6000.0f},
  };
  return (idx >= 0 && idx < 4) ? &pids[idx] : nullptr;
}

hello_world::pid::Pid *GetPidChassisFollow() {
  // 输入为云台相对底盘角误差，输出为底盘角速度 rad/s。
  static hello_world::pid::Pid pid(4.0f, 0.0f, 0.05f, 0.5f, 3.0f);
  return &pid;
}

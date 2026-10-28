/**
 *******************************************************************************
 * @file      : ins_robot.cpp
 * @brief     : 云台顶层状态机与运动模块单例
 *******************************************************************************
 */
#include "ins_robot.hpp"

#include "gimbal.hpp"
#include "robot.hpp"

robot::Robot *GetRobot() {
  static robot::Robot robot;
  return &robot;
}

robot::Gimbal *GetGimbal() {
  static robot::Gimbal gimbal;
  return &gimbal;
}

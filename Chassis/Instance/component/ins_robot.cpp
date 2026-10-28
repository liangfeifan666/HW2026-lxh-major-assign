/**
 *******************************************************************************
 * @file      : ins_robot.cpp
 * @brief     : 底盘顶层状态机与运动模块单例
 *******************************************************************************
 */
#include "ins_robot.hpp"

#include "chassis.hpp"
#include "robot.hpp"

robot::Robot *GetRobot() {
  static robot::Robot robot;
  return &robot;
}

robot::Chassis *GetChassis() {
  static robot::Chassis chassis;
  return &chassis;
}

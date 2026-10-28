/**
 *******************************************************************************
 * @file      : ins_chassis_gimbal_comm.cpp
 * @brief     : 底盘板 C2G=0x210，G2C=0x211
 *******************************************************************************
 */
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"

robot::GimbalChassisComm *GetGimbalChassisComm() {
  static robot::GimbalChassisComm comm(
      robot::GimbalChassisComm::CodePart::Chassis, 0x210, 0x211);
  return &comm;
}

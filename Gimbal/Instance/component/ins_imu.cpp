/**
 *******************************************************************************
 * @file      : ins_imu.cpp
 * @brief     : 本板 IMU 单例
 *******************************************************************************
 */
#include "ins_imu.hpp"

#include "imu.hpp"

hello_world::imu::Imu *GetImu() {
  static hello_world::imu::Imu imu;
  return &imu;
}

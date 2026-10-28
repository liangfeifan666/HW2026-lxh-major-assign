/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : BMI088 + Mahony 封装，零偏采样在 1ms 周期内非阻塞完成
 *******************************************************************************
 */
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

#include <cstdint>

#include "BMI088.hpp"
#include "mahony.hpp"

namespace hello_world {
namespace imu {

class Imu {
 public:
  bool init();
  void update();

  bool isReady() const { return initialized_ && calibrated_ && healthy_; }
  bool isCalibrated() const { return calibrated_; }
  uint16_t calibrationSamples() const { return sample_count_; }
  float roll() const { return euler_[0]; }
  float pitch() const { return euler_[1]; }
  float yaw() const { return euler_[2]; }

 private:
  static constexpr uint16_t kCalibrationSampleCount = 500;

  BMI088 sensor_;
  ahrs::Mahony ahrs_;
  float gyro_bias_[3] = {};
  float gyro_sum_[3] = {};
  float euler_[3] = {};
  uint16_t sample_count_ = 0;
  bool initialized_ = false;
  bool calibrated_ = false;
  bool healthy_ = false;
};

}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */

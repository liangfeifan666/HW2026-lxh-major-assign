/**
 *******************************************************************************
 * @file      : imu.cpp
 * @brief     : 静止零偏估计与 1kHz Mahony 姿态更新
 *******************************************************************************
 */
#include "imu.hpp"

#include <cmath>

#include "spi.h"

namespace hello_world {
namespace imu {

bool Imu::init() {
  BMI088HWConfig hw;
  hw.hspi = &hspi1;
  hw.acc_cs_port = GPIOA;
  hw.acc_cs_pin = GPIO_PIN_4;
  hw.gyro_cs_port = GPIOB;
  hw.gyro_cs_pin = GPIO_PIN_0;
  const float rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  sensor_.init(hw, rotation);
  initialized_ = sensor_.imuInit(false) == kBMI088ErrStateNoErr;
  calibrated_ = false;
  healthy_ = false;
  sample_count_ = 0;
  ahrs_.init(1000.0f, 0.25f, 0.0f);
  return initialized_;
}

void Imu::update() {
  if (!initialized_) return;

  float acc[3];
  float gyro[3];
  sensor_.getData(acc, gyro, nullptr);
  const float acc_norm = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] +
                                   acc[2] * acc[2]);
  healthy_ = std::isfinite(acc_norm) && acc_norm > 1.0f && acc_norm < 30.0f;
  for (int i = 0; i < 3; ++i) {
    healthy_ = healthy_ && std::isfinite(gyro[i]);
    gyro[i] -= gyro_bias_[i];
  }
  if (!healthy_) {
    calibrated_ = false;
    sample_count_ = 0;
    gyro_sum_[0] = gyro_sum_[1] = gyro_sum_[2] = 0.0f;
    return;
  }

  if (!calibrated_) {
    const bool stationary = std::fabs(gyro[0]) < 0.05f &&
                            std::fabs(gyro[1]) < 0.05f &&
                            std::fabs(gyro[2]) < 0.05f &&
                            acc_norm > 7.0f && acc_norm < 12.0f;
    if (!stationary) {
      sample_count_ = 0;
      gyro_sum_[0] = gyro_sum_[1] = gyro_sum_[2] = 0.0f;
      return;
    }

    for (int i = 0; i < 3; ++i) gyro_sum_[i] += gyro[i] + gyro_bias_[i];
    ++sample_count_;
    if (sample_count_ >= kCalibrationSampleCount) {
      for (int i = 0; i < 3; ++i) {
        gyro_bias_[i] = gyro_sum_[i] / kCalibrationSampleCount;
        gyro[i] -= gyro_bias_[i];
      }
      calibrated_ = true;
      ahrs_.init(1000.0f, 0.25f, 0.0f);
    } else {
      return;
    }
  }

  ahrs_.update(acc, gyro);
  ahrs_.getEulerAngle(euler_);
}

}  // namespace imu
}  // namespace hello_world

/**
 *******************************************************************************
 * @file      : imu_controller.cpp
 * @brief     : 静止零偏估计、实际采样间隔 Mahony 与一致姿态快照
 *******************************************************************************
 */
#include "imu_controller.hpp"

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
  ready_ = false;
  last_update_tick_ = HAL_GetTick();
  sample_count_ = 0;
  gyro_sum_[0] = gyro_sum_[1] = gyro_sum_[2] = 0.0f;
  ahrs_.init(1000.0f, 0.25f, 0.0f);
  return initialized_;
}

void Imu::update(uint32_t elapsed_ms) {
  if (!initialized_) return;
  if (elapsed_ms == 0) elapsed_ms = 1;
  if (elapsed_ms > 100) elapsed_ms = 100;

  float acc[3];
  float gyro[3];
  const bool read_ok = sensor_.getData(acc, gyro, nullptr);
  const float acc_norm = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] +
                                   acc[2] * acc[2]);
  healthy_ = read_ok && std::isfinite(acc_norm) &&
             acc_norm > 1.0f && acc_norm < 30.0f;
  for (int i = 0; i < 3; ++i) {
    healthy_ = healthy_ && std::isfinite(gyro[i]);
    gyro[i] -= gyro_bias_[i];
  }
  if (!healthy_) {
    ready_ = false;
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
        const float old_bias = gyro_bias_[i];
        gyro_bias_[i] = gyro_sum_[i] / kCalibrationSampleCount;
        gyro[i] += old_bias - gyro_bias_[i];
      }
      calibrated_ = true;
      ahrs_.init(1000.0f, 0.25f, 0.0f);
    } else {
      return;
    }
  }

  ahrs_.setSampleFrequency(1000.0f / static_cast<float>(elapsed_ms));
  ahrs_.update(acc, gyro);
  float new_euler[3];
  ahrs_.getEulerAngle(new_euler);
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  euler_[0] = new_euler[0];
  euler_[1] = new_euler[1];
  euler_[2] = new_euler[2];
  last_update_tick_ = HAL_GetTick();
  ready_ = calibrated_ && healthy_;
  __set_PRIMASK(primask);
}

}  // namespace imu
}  // namespace hello_world

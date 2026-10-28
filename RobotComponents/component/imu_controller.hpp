/**
 *******************************************************************************
 * @file      : imu_controller.hpp
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
  void update(uint32_t elapsed_ms = 1);

  bool isInitialized() const { return initialized_; }
  bool isReady() const {
    return ready_ && HAL_GetTick() - last_update_tick_ <= 100;
  }
  bool isCalibrated() const { return calibrated_; }
  uint16_t calibrationSamples() const { return sample_count_; }
  float roll() const {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const float value = euler_[0];
    __set_PRIMASK(primask);
    return value;
  }
  float pitch() const {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const float value = euler_[1];
    __set_PRIMASK(primask);
    return value;
  }
  float yaw() const {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const float value = euler_[2];
    __set_PRIMASK(primask);
    return value;
  }

 private:
  static constexpr uint16_t kCalibrationSampleCount = 500;

  BMI088 sensor_;
  ahrs::Mahony ahrs_;
  float gyro_bias_[3] = {};
  float gyro_sum_[3] = {};
  float euler_[3] = {};
  uint16_t sample_count_ = 0;
  volatile uint32_t last_update_tick_ = 0;
  volatile bool ready_ = false;
  bool initialized_ = false;
  bool calibrated_ = false;
  bool healthy_ = false;
};

}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */

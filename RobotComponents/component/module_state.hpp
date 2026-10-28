/**
 *******************************************************************************
 * @file      : module_state.hpp
 * @brief     : 双板共享的纯类型定义，不依赖任何硬件
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

#include <cmath>
#include <cstdint>

namespace robot {

enum class LifeState : uint8_t {
  Dead = 0,          // 上电、急停或任一关键输入失效：输出必须为零
  Resurrection = 1,  // 自检、IMU 零漂收敛与重新解锁
  Working = 2,
};

enum class ControlSource : uint8_t {
  None = 0,
  RemoteControl = 1,
};

enum class ChassisMode : uint8_t {
  Separate = 0,  // 分离：w 由拨轮直接给出
  Follow = 1,    // 跟随：w 由云台相对底盘角度闭环给出
  Spin = 2,      // 小陀螺：w 仍由拨轮给出并限幅，回中即零
};

enum class GimbalMode : uint8_t {
  JointSpace = 0,
};

// vx/vy 单位为 m/s，w 单位为 rad/s；平移指令始终定义在云台坐标系。
struct ChassisMotion {
  float vx = 0.0f;
  float vy = 0.0f;
  float w = 0.0f;

  constexpr ChassisMotion operator+(const ChassisMotion &rhs) const {
    return {vx + rhs.vx, vy + rhs.vy, w + rhs.w};
  }
  constexpr ChassisMotion operator-(const ChassisMotion &rhs) const {
    return {vx - rhs.vx, vy - rhs.vy, w - rhs.w};
  }
  constexpr ChassisMotion operator*(float scale) const {
    return {vx * scale, vy * scale, w * scale};
  }
  ChassisMotion &operator+=(const ChassisMotion &rhs) {
    vx += rhs.vx;
    vy += rhs.vy;
    w += rhs.w;
    return *this;
  }
};

// 单位为 rad；yaw 为连续目标角，电机反馈角通过 WrapPi 处理回绕。
struct GimbalMotion {
  float yaw = 0.0f;
  float pitch = 0.0f;

  constexpr GimbalMotion operator+(const GimbalMotion &rhs) const {
    return {yaw + rhs.yaw, pitch + rhs.pitch};
  }
  constexpr GimbalMotion operator-(const GimbalMotion &rhs) const {
    return {yaw - rhs.yaw, pitch - rhs.pitch};
  }
  constexpr GimbalMotion operator*(float scale) const {
    return {yaw * scale, pitch * scale};
  }
  GimbalMotion &operator+=(const GimbalMotion &rhs) {
    yaw += rhs.yaw;
    pitch += rhs.pitch;
    return *this;
  }
};

constexpr float kPi = 3.14159265358979323846f;

inline float WrapPi(float angle) {
  // remainder 有界执行；异常浮点直接归零，避免中断里死循环。
  return std::isfinite(angle) ? std::remainder(angle, 2.0f * kPi) : 0.0f;
}

inline float Clamp(float value, float lower, float upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

}  // namespace robot

#endif /* MODULE_STATE_HPP_ */

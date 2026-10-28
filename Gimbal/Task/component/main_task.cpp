/**
 *******************************************************************************
 * @file      : main_task.cpp
 * @brief     : 云台对象装配与 1ms 控制入口
 *******************************************************************************
 */
#include "main_task.hpp"

#include "comm_task.hpp"
#include "gimbal.hpp"
#include "iwdg.h"
#include "ins_all.hpp"
#include "motor_DaMiao.hpp"
#include "robot.hpp"
#include "tim.h"

namespace {
volatile bool g_init_done = false;
}
volatile bool g_cycle_completed = false;
volatile uint32_t g_background_tick = 0;

void MainTaskInit(void) {
  __HAL_DBGMCU_FREEZE_IWDG();
  auto *robot = GetRobot();
  auto *gimbal = GetGimbal();
  auto *yaw = GetMotorYaw();
  auto *pitch = GetMotorPitch();
  yaw->setInput(0.0f);
  pitch->setInput(0.0f);
  gimbal->registerYaw(yaw, GetPidMotorYaw());
  gimbal->registerPitch(pitch, GetPidMotorPitch());
  robot->registerGimbal(gimbal);
  robot->registerImu(GetImu());
  robot->registerComm(GetGimbalChassisComm());
  robot->registerRemote(GetRemoteControl(), GetRemoteControlOfflineChecker());
  robot->registerMotors(yaw, pitch, GetMotorYawOfflineChecker(),
                        GetMotorPitchOfflineChecker());
  GetImu()->init();
  CommTaskInit();
  // 仅在初始化阶段、零转矩下显式使能一次；运行故障后不自动重新武装。
  static_cast<hello_world::motor::DaMiao *>(yaw)->enable();
  static_cast<hello_world::motor::DaMiao *>(pitch)->enable();
  g_init_done = true;
}

void MainTask(void) {
  if (!g_init_done) return;
  g_cycle_completed = false;
  GetRobot()->update();
  g_cycle_completed = true;
}

void MainTaskBackground(void) {
  if (!g_init_done) return;
  const uint32_t now = HAL_GetTick();
  g_background_tick = now;
  auto *imu = GetImu();
  static uint32_t last_init_attempt = 0;
  static uint32_t last_sample = 0;
  if (!imu->isInitialized()) {
    if (now - last_init_attempt >= 100) {
      last_init_attempt = now;
      imu->init();
    }
    return;
  }
  const uint32_t elapsed = now - last_sample;
  if (elapsed >= 1) {
    last_sample = now;
    imu->update(elapsed);
  }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim != &htim6 || !g_init_done) return;
  MainTask();
  CommTask();
}

extern volatile bool g_cycle_completed;

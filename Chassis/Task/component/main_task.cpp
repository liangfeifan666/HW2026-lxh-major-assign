/**
 *******************************************************************************
 * @file      : main_task.cpp
 * @brief     : 底盘对象装配与 1ms 控制入口
 *******************************************************************************
 */
#include "main_task.hpp"

#include "comm_task.hpp"
#include "iwdg.h"
#include "ins_all.hpp"
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
  auto *chassis = GetChassis();
  chassis->registerIkSolver(GetChassisIkSolver());
  robot->registerChassis(chassis);
  robot->registerImu(GetImu());
  robot->registerComm(GetGimbalChassisComm());
  robot->registerFollowPid(GetPidChassisFollow());
  for (int i = 0; i < 4; ++i) {
    auto *motor = GetMotorWheel(i);
    motor->setInput(0.0f);
    robot->registerWheel(i, motor, GetPidMotorWheel(i),
                         GetMotorWheelOfflineChecker(i));
  }
  GetImu()->init();
  CommTaskInit();
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
      imu->init();  // 后台限频重试，可被 TIM6 安全中断抢占。
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

// comm_task.cpp 仅在完整控制周期结束后读取并清除该标志。
extern volatile bool g_cycle_completed;

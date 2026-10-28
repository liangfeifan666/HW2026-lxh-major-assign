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
#include "robot.hpp"
#include "tim.h"

namespace {
volatile bool g_init_done = false;
}
volatile bool g_cycle_completed = false;

void MainTaskInit(void) {
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
  __HAL_DBGMCU_FREEZE_IWDG();
  g_init_done = true;
}

void MainTask(void) {
  if (!g_init_done) return;
  g_cycle_completed = false;
  GetRobot()->update();
  g_cycle_completed = true;
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim != &htim6 || !g_init_done) return;
  MainTask();
  CommTask();
}

extern volatile bool g_cycle_completed;

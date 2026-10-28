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

void MainTaskInit(void) {
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
  GetRemoteControl();  // 在开启 UART 中断前完成静态对象构造。
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

// comm_task.cpp 仅在完整控制周期结束后读取并清除该标志。
extern volatile bool g_cycle_completed;

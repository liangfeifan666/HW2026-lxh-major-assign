/**
 *******************************************************************************
 * @file      : comm_task.cpp
 * @brief     : 底盘 CAN 通信；启动失败限周期重试，遥控只来自云台板
 *******************************************************************************
 */
#include "comm_task.hpp"

#include <cstring>

#include "HW_can.hpp"
#include "can_lifecycle.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "iwdg.h"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"
#include "motor_M3508.hpp"

extern volatile bool g_cycle_completed;
extern volatile uint32_t g_background_tick;

namespace {
bool g_comm_initialized = false;
volatile bool g_can1_ready = false;
volatile bool g_can2_ready = false;
volatile bool g_motor_tx_ok = false;
}

void CommTaskInit(void) {
  g_can1_ready = robot::CanStart(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  g_can2_ready = robot::CanStart(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_IWDG_Refresh(&hiwdg);
  g_comm_initialized = true;
}

void CommTask(void) {
  if (!g_comm_initialized) return;
  g_can1_ready = g_can1_ready
                     ? robot::CanRecoverBusOff(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
                     : robot::CanStart(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  g_can2_ready = g_can2_ready
                     ? robot::CanRecoverBusOff(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING)
                     : robot::CanStart(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  uint8_t wheel_data[8] = {};
  bool encoded = true;
  for (int i = 0; i < 4; ++i) {
    encoded = GetMotorWheel(i)->encode(wheel_data) && encoded;
  }
  if (!encoded) std::memset(wheel_data, 0, sizeof(wheel_data));
  g_motor_tx_ok = CAN_Send_Msg(
                      &hcan1, wheel_data,
                      hello_world::motor::M3508::GetTxId(1), 8) == HAL_OK;

  uint8_t comm_data[8];
  auto *comm = GetGimbalChassisComm();
  comm->encode(comm_data);
  CAN_Send_Msg(&hcan2, comm_data, comm->tx_id(), 8);

  if (g_cycle_completed && HAL_GetTick() - g_background_tick <= 100) {
    HAL_IWDG_Refresh(&hiwdg);
    g_cycle_completed = false;
  }
}

extern "C" bool CommMotorCanHealthy(void) {
  return g_can1_ready && g_motor_tx_ok &&
         (hcan1.Instance->ESR & CAN_ESR_BOFF) == 0;
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  if ((hcan->ErrorCode & HAL_CAN_ERROR_BOF) == 0) return;
  if (hcan == &hcan1) {
    g_can1_ready = robot::CanRecoverBusOff(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  } else if (hcan == &hcan2) {
    g_can2_ready = robot::CanRecoverBusOff(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  }
}

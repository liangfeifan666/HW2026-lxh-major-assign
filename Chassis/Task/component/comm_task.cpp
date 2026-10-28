/**
 *******************************************************************************
 * @file      : comm_task.cpp
 * @brief     : 底盘 CAN/UART 通信；四轮共享一帧，安全态仍持续发送
 *******************************************************************************
 */
#include "comm_task.hpp"

#include <cstring>

#include "HW_can.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "iwdg.h"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "motor_M3508.hpp"
#include "usart.h"

extern volatile bool g_cycle_completed;

namespace {
uint8_t g_rc_rx_data[18];
bool g_comm_initialized = false;

void ArmRemoteRx() {
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_rc_rx_data, 18);
  // 禁止半传输事件，回调中的短帧就只可能是一次需要重对齐的空闲帧。
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}
}

void CommTaskInit(void) {
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);
  (void)HAL_CAN_Start(&hcan1);
  (void)HAL_CAN_Start(&hcan2);
  (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  (void)HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  ArmRemoteRx();
  HAL_IWDG_Refresh(&hiwdg);
  g_comm_initialized = true;
}

void CommTask(void) {
  if (!g_comm_initialized) return;

  uint8_t wheel_data[8] = {};
  bool encoded = true;
  for (int i = 0; i < 4; ++i) {
    encoded = GetMotorWheel(i)->encode(wheel_data) && encoded;
  }
  if (!encoded) std::memset(wheel_data, 0, sizeof(wheel_data));
  CAN_Send_Msg(&hcan1, wheel_data,
               hello_world::motor::M3508::GetTxId(1), 8);

  uint8_t comm_data[8];
  auto *comm = GetGimbalChassisComm();
  comm->encode(comm_data);
  CAN_Send_Msg(&hcan2, comm_data, comm->tx_id(), 8);

  if (g_cycle_completed) {
    HAL_IWDG_Refresh(&hiwdg);
    g_cycle_completed = false;
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) return;
  // 每次空闲/满帧都重新启动，短帧后的下一帧从头对齐，禁止跨帧拼接。
  (void)HAL_UART_DMAStop(huart);
  if (Size == 18) DecodeRemoteControlFrame(g_rc_rx_data);
  ArmRemoteRx();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart3) return;
  (void)HAL_UART_DMAStop(huart);
  ArmRemoteRx();
}

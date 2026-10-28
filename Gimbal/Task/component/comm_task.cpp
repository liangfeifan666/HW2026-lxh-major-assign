/**
 *******************************************************************************
 * @file      : comm_task.cpp
 * @brief     : 云台 CAN/UART 通信；DM 编码异常时发送零转矩兜底帧
 *******************************************************************************
 */
#include "comm_task.hpp"

#include "HW_can.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "iwdg.h"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "motor_DaMiao.hpp"
#include "usart.h"

extern volatile bool g_cycle_completed;

namespace {
uint8_t g_rc_rx_data[18];
bool g_comm_initialized = false;

void ArmRemoteRx() {
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_rc_rx_data, 18);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void SendMotor(hello_world::motor::Motor *motor) {
  uint8_t data[8] = {};
  auto *dm = static_cast<hello_world::motor::DaMiao *>(motor);
  if (motor->encode(data)) {
    CAN_Send_Msg(&hcan1, data, dm->tx_id(), 8);
  } else {
    // MIT：pos=0、vel=0、kp=0、kd=0、torque=0。
    const uint8_t zero_torque[8] = {0x7F, 0xFF, 0x7F, 0xF0,
                                    0x00, 0x00, 0x07, 0xFF};
    CAN_Send_Msg(&hcan1, const_cast<uint8_t *>(zero_torque), motor->id(), 8);
  }
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
  SendMotor(GetMotorYaw());
  SendMotor(GetMotorPitch());

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
  // 短帧后重新从帧头接收，避免把两帧各一段拼成伪有效遥控数据。
  (void)HAL_UART_DMAStop(huart);
  if (Size == 18) DecodeRemoteControlFrame(g_rc_rx_data);
  ArmRemoteRx();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart3) return;
  (void)HAL_UART_DMAStop(huart);
  ArmRemoteRx();
}

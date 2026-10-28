/**
 *******************************************************************************
 * @file      : comm_task.cpp
 * @brief     : 云台 CAN/UART 通信；外设启动失败在后续周期重试
 *******************************************************************************
 */
#include "comm_task.hpp"

#include "HW_can.hpp"
#include "can_lifecycle.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "iwdg.h"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "motor_DaMiao.hpp"
#include "usart.h"

extern volatile bool g_cycle_completed;
extern volatile uint32_t g_background_tick;

namespace {
uint8_t g_rc_rx_data[18];
bool g_comm_initialized = false;
volatile bool g_can1_ready = false;
volatile bool g_can2_ready = false;
volatile bool g_remote_rx_ready = false;
volatile bool g_motor_tx_ok = false;

bool ArmRemoteRx() {
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_rc_rx_data, 18) != HAL_OK) {
    return false;
  }
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  return true;
}

bool SendMotor(hello_world::motor::Motor *motor) {
  uint8_t data[8] = {};
  auto *dm = static_cast<hello_world::motor::DaMiao *>(motor);
  if (motor->encode(data)) {
    return CAN_Send_Msg(&hcan1, data, dm->tx_id(), 8) == HAL_OK;
  }
  uint8_t zero_torque[8] = {0x80, 0x00, 0x80, 0x00,
                            0x00, 0x00, 0x08, 0x00};
  return CAN_Send_Msg(&hcan1, zero_torque, motor->id(), 8) == HAL_OK;
}
}

void CommTaskInit(void) {
  g_can1_ready = robot::CanStart(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  g_can2_ready = robot::CanStart(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  g_remote_rx_ready = ArmRemoteRx();
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
  if (!g_remote_rx_ready) g_remote_rx_ready = ArmRemoteRx();

  const bool yaw_tx_ok = SendMotor(GetMotorYaw());
  const bool pitch_tx_ok = SendMotor(GetMotorPitch());
  g_motor_tx_ok = yaw_tx_ok && pitch_tx_ok;
  uint8_t comm_data[8];
  auto *comm = GetGimbalChassisComm();
  comm->encode(comm_data);
  CAN_Send_Msg(&hcan2, comm_data, comm->tx_id(), 8);

  if (g_cycle_completed && HAL_GetTick() - g_background_tick <= 100) {
    HAL_IWDG_Refresh(&hiwdg);
    g_cycle_completed = false;
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) return;
  g_remote_rx_ready = false;
  (void)HAL_UART_DMAStop(huart);
  if (Size == 18) DecodeRemoteControlFrame(g_rc_rx_data);
  g_remote_rx_ready = ArmRemoteRx();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart3) return;
  g_remote_rx_ready = false;
  (void)HAL_UART_DMAStop(huart);
  g_remote_rx_ready = ArmRemoteRx();
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

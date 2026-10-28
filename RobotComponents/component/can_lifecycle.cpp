/**
 *******************************************************************************
 * @file      : can_lifecycle.cpp
 * @brief     : bxCAN 硬件 ABOM 恢复；不在故障路径忙等
 *******************************************************************************
 */
#include "can_lifecycle.hpp"

#include "HW_can.hpp"

namespace robot {

bool CanStart(CAN_HandleTypeDef *hcan, uint32_t rx_interrupt) {
  if ((hcan->Instance->ESR & CAN_ESR_BOFF) != 0) return false;
  if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING) {
    if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_ERROR &&
        HAL_CAN_Init(hcan) != HAL_OK) return false;
    CanFilter_Init(hcan);
    if (HAL_CAN_Start(hcan) != HAL_OK) return false;
  }
  SET_BIT(hcan->Instance->MCR, CAN_MCR_ABOM);
  return HAL_CAN_ActivateNotification(
             hcan, rx_interrupt | CAN_IT_ERROR | CAN_IT_BUSOFF) == HAL_OK;
}

bool CanRecoverBusOff(CAN_HandleTypeDef *hcan, uint32_t rx_interrupt) {
  if ((hcan->Instance->ESR & CAN_ESR_BOFF) == 0) return true;
  SET_BIT(hcan->Instance->MCR, CAN_MCR_ABOM);
  HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 |
                                   CAN_TX_MAILBOX2);
  hcan->ErrorCode = HAL_CAN_ERROR_NONE;
  (void)HAL_CAN_ActivateNotification(
      hcan, rx_interrupt | CAN_IT_ERROR | CAN_IT_BUSOFF);
  return false;  // BOFF 尚未由硬件恢复序列清除，下一周期继续检查。
}

}  // namespace robot

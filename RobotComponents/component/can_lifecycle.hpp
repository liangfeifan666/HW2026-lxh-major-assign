/**
 *******************************************************************************
 * @file      : can_lifecycle.hpp
 * @brief     : 双板统一的 CAN 启动、通知恢复与 bus-off 处理
 *******************************************************************************
 */
#ifndef CAN_LIFECYCLE_HPP_
#define CAN_LIFECYCLE_HPP_

#include <cstdint>

#include "stm32_hal.hpp"

namespace robot {

bool CanStart(CAN_HandleTypeDef *hcan, uint32_t rx_interrupt);
bool CanRecoverBusOff(CAN_HandleTypeDef *hcan, uint32_t rx_interrupt);

}  // namespace robot

#endif

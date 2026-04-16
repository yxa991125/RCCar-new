/**
 * @file servo_basic_task.h
 * @brief FreeRTOS task bridge for servo basic control.
 */

#ifndef SERVO_BASIC_TASK_H
#define SERVO_BASIC_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "FreeRTOS.h"

void ServoBasic_TaskInit(void);
void ServoBasic_Task(void *param);
BaseType_t ServoBasic_EnqueueCanMessageFromISR(uint32_t can_id,
											   const uint8_t *payload,
											   uint8_t payload_len,
											   BaseType_t *pxHigherPriorityTaskWoken);

#ifdef __cplusplus
}
#endif

#endif

/*
 * m18_operations.h
 *
 *  Created on: Mar 13, 2025
 *      Author: ZacV
 */

#ifndef INC_M18_OPERATIONS_H
#define INC_M18_OPERATIONS_H

#include "stm32h7xx_hal.h"
#include "globals.h"

typedef struct {
	GPIO_TypeDef* port;
	uint32_t pin;
} PinConfig;
typedef struct {
	volatile uint32_t m18StartTime;
	volatile bool m18InProcess;
	volatile PinConfig activePin;
} M18OperationState;



#endif /* INC_M18_OPERATIONS_H */


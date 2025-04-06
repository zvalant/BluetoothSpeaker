/*
 * m18_operations.h
 *
 *  Created on: Mar 13, 2025
 *      Author: ZacV
 */

#ifndef INC_M18_OPERATIONS_H
#define INC_M18_OPERATIONS_H


//Standard Header Files
#include "stm32h7xx_hal.h"
#include "globals.h"

//Project Header Files
#include "m18_state.h"

void inputOutputPinAssignment(PinConfig currentPin);
void m18TaskTrigger(void);
uint8_t m18TaskCompletionCheck(void);
uint8_t m18Call(inputState currentState);


#endif /* INC_M18_OPERATIONS_H */


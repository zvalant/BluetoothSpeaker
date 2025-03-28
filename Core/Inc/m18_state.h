/*
 * m18_state.h
 *
 *  Created on: Mar 27, 2025
 *      Author: zac23
 */

#ifndef INC_M18_STATE_H_
#define INC_M18_STATE_H_


#include "stm32h7xx_hal.h"
#include "stdbool.h"
//Project specific includes
#include "globals.h"



//Keep track of current m18 state
typedef struct {
	GPIO_TypeDef* port;
	uint32_t pin;
} PinConfig;
typedef struct {
	volatile uint32_t m18StartTime;
	volatile bool m18InProcess;
	volatile PinConfig activePin;
} M18OperationState;

extern M18OperationState m18State;
extern M18OperationState* m18StatePtr;

void m18SetStartTime(void);
uint32_t m18GetStartTime(void);
void m18SetInProcess(bool isActive);
bool m18IsProcessActive(void);
PinConfig m18GetActivePin(void);
void m18SetActivePin(inputState currentState);



#endif /* INC_M18_STATE_H_ */

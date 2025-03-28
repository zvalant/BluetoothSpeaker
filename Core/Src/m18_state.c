/*
 * m18_state.c
 *
 *  Created on: Mar 27, 2025
 *      Author: zac23
 */


//Standard library
#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "gpio.h"
#include "stdbool.h"
//Project specific
#include "m18_state.h"
#include "globals.h"




M18OperationState m18State = {0, false,(PinConfig){TRACK_OPTIONS_PORT, PAUSE_PLAY_PIN}};
M18OperationState* m18StatePtr = &m18State;



void m18SetStartTime(void){
	m18StatePtr->m18StartTime = HAL_GetTick();
}

uint32_t m18GetStartTime(void){
	return m18StatePtr->m18StartTime;
}

void m18SetInProcess(bool isActive){
	m18StatePtr->m18InProcess = isActive;

}
bool m18IsProcessActive(void){
	return m18StatePtr->m18InProcess;

}

PinConfig m18GetActivePin(void){
	return m18StatePtr->activePin;
}

void m18SetActivePin(inputState currentState){
		switch(currentState){
			case STATE_PAUSE_PLAY:
				m18StatePtr->activePin = (PinConfig){TRACK_OPTIONS_PORT, PAUSE_PLAY_PIN};
				break;
			case STATE_PREV_TRACK:
				m18StatePtr->activePin = (PinConfig){TRACK_OPTIONS_PORT, PREV_TRACK_PIN};
				break;
			case STATE_NEXT_TRACK:
				m18StatePtr->activePin = (PinConfig){TRACK_OPTIONS_PORT, NEXT_TRACK_PIN};
				break;
			case STATE_POWER_OFF_ON:
				m18StatePtr->activePin = (PinConfig){M18_POWER_PORT, M18_POWER_PIN};
				break;
			default:
				break;
		}
}






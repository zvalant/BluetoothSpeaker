/*
 * m18_operations.c
 *v0.0.1
 *  Created on: Mar 13, 2025
 *      Author: ZacV
 */
//std header files
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "usart.h"

//project header files
#include "m18_operations.h"
#include "globals.h"




//CONSTS
const uint32_t M18_DELAY_MS = 50;
const uint8_t M18_CALL_INVALID = -2;
const uint8_t M18_CALL_DENIED = -1;
const uint8_t M18_CALL_SUCCESS = 0;
const uint8_t M18_TASK_INCOMPLETE = -1;
const uint8_t M18_TASK_COMPLETE = 0;
const uint8_t M18_NO_TASK_IN_PROCESS = 0;
//m18 state & ptr
M18OperationState m18State = {0, false,(PinConfig){TRACK_OPTIONS_PORT, PAUSE_PLAY_PIN}};
M18OperationState* m18StatePtr = &m18State;



/*InputOutputPinAssignment: Will change target pin to either an input if the m18 task is complete
 * or an output to properly execute m18 task.
 *
 */
void InputOutputPinAssignment(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = m18StatePtr->activePin.pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	if (m18StatePtr->m18InProcess){
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

	}else{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	HAL_GPIO_Init(m18StatePtr->activePin.port, &GPIO_InitStruct);
	if (!m18StatePtr->m18InProcess){
		HAL_GPIO_WritePin(m18StatePtr->activePin.port, m18StatePtr->activePin.pin, GPIO_PIN_SET);
	}
	HAL_Delay(10);


}/*m18TaskTrigger: Changes the GPIO pin to an output then set pin to low to start the m18 function call process
and track the time of the trigger to be used to bring the pin high after a delay.
*
*/
void m18TaskTrigger(void){
	InputOutputPinAssignment();
	HAL_GPIO_WritePin(m18StatePtr->activePin.port,m18StatePtr->activePin.pin,GPIO_PIN_RESET);
	m18StatePtr->m18InProcess = true;
	m18StatePtr->m18StartTime = HAL_GetTick();
}
/*m18TaskCompletionCheck: Called in a non idle state and will debounce if m18 task isnt in process.
 * After required delay the pin is set back to high ending m18 call and finish logic to complete m18 call process.
 *
 */
uint8_t m18TaskCompletionCheck(void){
	if (!m18StatePtr->m18InProcess){
		return M18_NO_TASK_IN_PROCESS;

	}
	if (HAL_GetTick()>m18StatePtr->m18StartTime+M18_DELAY_MS){
		HAL_GPIO_WritePin(m18StatePtr->activePin.port, m18StatePtr->activePin.pin,GPIO_PIN_SET);
		InputOutputPinAssignment();
		m18StatePtr->m18InProcess = false;
		return M18_TASK_COMPLETE;
	}
	 return M18_TASK_INCOMPLETE;

}
/*
 * m18Call will either debounce if m18 process is active
 * or will start a m18 function call depending on passed state
 *
 */
uint8_t m18Call(inputState currentState){
	if (m18StatePtr->m18InProcess){
		return M18_CALL_DENIED;
	}
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
			return M18_CALL_INVALID;
	}
	m18TaskTrigger();
	return M18_CALL_SUCCESS;

}

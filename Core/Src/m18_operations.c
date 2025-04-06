/*
 * m18_operations.c
 *v0.0.1
 *  Created on: Mar 13, 2025
 *      Author: ZacV
 */
//Standard Header Files
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "usart.h"

//Project Header Files
#include "m18_operations.h"
#include "globals.h"
#include "m18_state.h"
#include "user_button.h"




//CONSTS
const uint32_t M18_DELAY_MS = 50;
const uint8_t M18_CALL_INVALID = -2;
const uint8_t M18_CALL_DENIED = -1;
const uint8_t M18_CALL_SUCCESS = 0;
const uint8_t M18_TASK_INCOMPLETE = -1;
const uint8_t M18_TASK_COMPLETE = 0;
const uint8_t M18_NO_TASK_IN_PROCESS = 0;



/*InputOutputPinAssignment: Will change target pin to either an input if the m18 task is complete
 * or an output to properly execute m18 task.
 *
 */
void inputOutputPinAssignment(PinConfig currentPin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	bool m18Active = m18IsProcessActive();
	GPIO_InitStruct.Pin = currentPin.pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	if (m18Active){
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

	}else{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	HAL_GPIO_Init(currentPin.port, &GPIO_InitStruct);
	if (!m18Active){
		HAL_GPIO_WritePin(currentPin.port,currentPin.pin, GPIO_PIN_SET);
	}
	HAL_Delay(10);


}
/*m18TaskTrigger: Changes the GPIO pin to an output then set pin to low to start the m18 function call process
and track the time of the trigger to be used to bring the pin high after a delay.
*
*/
void m18TaskTrigger(void){
	disableUserButton();
	PinConfig currentPin = m18GetActivePin();
	inputOutputPinAssignment(currentPin);
	m18SetInProcess(true);
	HAL_GPIO_WritePin(currentPin.port,currentPin.pin,GPIO_PIN_RESET);
	m18SetStartTime();
}
/*m18TaskCompletionCheck: Called in a non idle state and will debounce if m18 task isnt in process.
 * After required delay the pin is set back to high ending m18 call and finish logic to complete m18 call process.
 *
 */
uint8_t m18TaskCompletionCheck(void){
	if (!m18IsProcessActive()){
		return M18_NO_TASK_IN_PROCESS;

	}
	if (HAL_GetTick()>m18GetStartTime()+M18_DELAY_MS){
		PinConfig currentPin = m18GetActivePin();
		HAL_GPIO_WritePin(currentPin.port, currentPin.pin,GPIO_PIN_SET);
		inputOutputPinAssignment(currentPin);
		m18SetInProcess(false);
		enableUserButton();

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
	if (m18IsProcessActive()){
		return M18_CALL_DENIED;
	}
	m18SetActivePin(currentState);
	m18TaskTrigger();
	return M18_CALL_SUCCESS;

}

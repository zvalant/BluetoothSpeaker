/*
 * speaker_state.c
 *
 *  Created on: Mar 25, 2025
 *      Author: zac23
 */
//Standard Header Files
#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "gpio.h"
//Project Header Files
#include "speaker_state.h"
#include "globals.h"

State activeState = { 0 };
State *activeStatePtr = &activeState;

void updateSpeakerState(uint8_t newState) {
	activeStatePtr->currentState = newState;
	return;

}
uint8_t getSpeakerState(void) {
	return activeStatePtr->currentState;
}

void updateLastPress(void) {
	activeStatePtr->lastPress = HAL_GetTick();

}
uint32_t getLastPress(void) {
	return activeStatePtr->lastPress;
}

//

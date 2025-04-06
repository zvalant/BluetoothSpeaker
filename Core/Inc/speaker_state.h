/*
 * speaker_state.h
 *
 *  Created on: Mar 25, 2025
 *      Author: zac23
 */

#ifndef INC_SPEAKER_STATE_H_
#define INC_SPEAKER_STATE_H_

//Standard Header Files
#include "stm32h7xx_hal.h"
#include "stdint.h"

//Project Header Files
#include "globals.h"

//Keeps track of current speaker state
typedef struct{
	volatile inputState currentState;
	volatile uint32_t lastPress;

} State;
// Changes current speaker state with updated state
void updateSpeakerState(uint8_t newState);
// Retrieves current speaker state
uint8_t getSpeakerState(void);
// Stores current timestamp of rising edge of user button interrupt
void updateLastPress(void);
// Retrieves the timestamp of last press
uint32_t getLastPress(void);

extern State activeState;
extern State* activeStatePtr;



#endif /* INC_SPEAKER_STATE_H_ */

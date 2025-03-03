#ifndef GLOBALS_H
#define GLOBALS_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
// user input button
#define USER_BUTTON_PIN GPIO_PIN_13
#define USER_BUTTON_PORT GPIOC
// M18 KEY INPUTS
#define PAUSE_PLAY_PIN GPIO_PIN_5
#define NEXT_TRACK_PIN GPIO_PIN_6
#define PREV_TRACK_PIN GPIO_PIN_15
#define TRACK_OPTIONS_PORT GPIOA
#define M18_POWER_PIN GPIO_PIN_7
#define M18_POWER_PORT GPIOB
// STATE CHANGES
typedef enum{
	STATE_IDLE,
	STATE_PAUSE_PLAY,
	STATE_PREV_TRACK,
	STATE_NEXT_TRACK,
	STATE_POWER_OFF_ON
} inputState;
// GPIO INFO FOR STATE CHANGES
typedef struct {
	uint16_t pin;
	GPIO_TypeDef *port;

} PinConfig;
// MAIN STATE STRUCTURE
typedef struct {
	inputState currentState;
	uint32_t lastPress;
	PinConfig currentPinInfo;
} State;

extern State activeState;
extern State* activeStatePtr;
#endif


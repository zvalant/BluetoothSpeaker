/*
 * user_button.c
 *
 *  Created on: Apr 6, 2025
 *      Author: zac23
 */


//Standard Header Files
#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "gpio.h"
#include "stdbool.h"
//Project Header Files
#include "globals.h"
#include "user_button.h"


/*disableUserButton: Will mask interrupts for the user button to prevent unwanted
 * interrupts during active calls to receiver.
 */
void disableUserButton(void){
	EXTI->IMR1 &= ~EXTI_IMR1_IM13;

}
/*enableUserButton: Will clear bit to prevent stale interrupts from triggering
 * and will enable interrupts on user button again.
 *
 */
void enableUserButton(void){
	EXTI->PR1 = EXTI_PR1_PR13;
	EXTI->IMR1 |= EXTI_IMR1_IM13;
}


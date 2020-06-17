/*
 * button_fsm.c
 *
 *  Created on: Jun 16, 2020
 *      Author: juanf
 */
#include"button_fsm.h"

/* -Paste here #defines of button (led for testing) port & pin- */
#define BUT_Pin 		GPIO_PIN_13
#define BUT_GPIO_Port 	GPIOC

#define LED_Pin 		GPIO_PIN_5
#define LED_GPIO_Port 	GPIOA
/* ------------------------------------------ */
void but_fsm_init ( but_fsm_t *fsm ) {
	fsm->countdown = 0;
	fsm->event = EVT_BUT_NO_EVT;
	fsm->state = STATE_BUT_WAITING;
	fsm->new_event = false;
	fsm->start_countdown = false;
}

bool_t but_pressed ( void ) {
	return !HAL_GPIO_ReadPin(BUT_GPIO_Port, BUT_Pin); //Button in STM32L476RG is connected pull up
}

void but_fsm_run ( but_fsm_t *fsm ) {
	if (fsm->new_event == true) {
		fsm->new_event = false;

		switch (fsm->state) {

		case STATE_BUT_WAITING:
			if( fsm->event == EVT_BUT_PRESSED ) {
				fsm->start_countdown = true;
				fsm->state = STATE_BUT_DETECTED;
			}
			break;

		case STATE_BUT_DETECTED:
			if( fsm->event == EVT_BUT_TIMEOUT ) {
				if (but_pressed()) {
					fsm->start_countdown = true;
					fsm->state = STATE_BUT_WAIT_RELEASE;
				}
			}
			break;

		case STATE_BUT_WAIT_RELEASE:
			if ( fsm->event == EVT_BUT_TIMEOUT ) {
				if (!but_pressed()) {
					fsm->state = STATE_BUT_UPDATE;
					fsm->start_countdown = true;
				} else {
					fsm->state = STATE_BUT_WAIT_RELEASE;
					fsm->start_countdown = true;
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
				}
			}
			break;

		case STATE_BUT_UPDATE:
			if ( fsm->event == EVT_BUT_TIMEOUT ) {
				fsm->state = STATE_BUT_WAITING;
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
			}
			break;

		default: /* ERROR */

			while(1);

		}

	}
}


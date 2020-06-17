/*
 * button_fsm.h
 *
 *  Created on: Jun 16, 2020
 *      Author: juanf
 */

#ifndef INC_BUTTON_FSM_H_
#define INC_BUTTON_FSM_H_

#define TICK	(	 10  	)

#include "stm32l4xx_hal.h"
#include <stdbool.h>

typedef bool bool_t;

typedef enum _but_states_ {
	STATE_BUT_WAITING,
	STATE_BUT_DETECTED,
	STATE_BUT_WAIT_RELEASE,
	STATE_BUT_UPDATE,
	STATE_BUT_TOTAL,
}but_states_t;

typedef enum _but_events_ {
	EVT_BUT_NO_EVT,
	EVT_BUT_PRESSED,
	EVT_BUT_TIMEOUT,
	EVT_BUT_TOTAL,
}but_events_t;

typedef struct _but_fsm_ {
	uint16_t countdown;
	but_states_t state;
	but_events_t event;
	bool_t new_event:1;
	bool_t start_countdown:1;
}but_fsm_t;

void but_fsm_init ( but_fsm_t *fsm );
bool_t but_pressed ( void );
void but_fsm_run ( but_fsm_t *fsm );

#endif /* INC_BUTTON_FSM_H_ */

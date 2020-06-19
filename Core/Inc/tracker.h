/*
 * tracker.h
 *
 *  Created on: 18/06/2020
 *      Author: juanf
 */

#ifndef INC_TRACKER_H_
#define INC_TRACKER_H_

#include "stm32l4xx_hal.h"
#include "button_fsm.h"
#include "nmea.h"
#include <string.h>

#define TIME_TO_START_SYSTEM	(	   5000		)
#define TIME_NEEDED_TO_ALARM	(		200		)

typedef bool bool_t;

typedef enum _tracker_states_ {
	STATE_TRACKER_IDLE,
	STATE_TRACKER_WAIT_FOR_REQUEST_GPS,
	STATE_TRACKER_SEND_REQUEST_GPS,
	STATE_TRACKER_WAITING_GPS_RESPONSE,
	STATE_TRACKER_RECEIVED_GPS,
	STATE_TRACKER_GSM_SENDING,
}tracker_states_t;

typedef enum _tracker_events_ {
	EVT_TRACKER_NO_EVT,
	EVT_TRACKER_ALERT_ON,
	EVT_TRACKER_TIMEOUT_REQUEST_GPS,
	EVT_TRACKER_GPS_REQUEST_SENT,
	EVT_TRACKER_TIMEOUT_FAIL,
	EVT_TRACKER_GPS_RESPONSE_RECEIVED,
	EVT_TRACKER_NO_CORRECT_DATA,
	EVT_TRACKER_CORRECT_DATA,
	EVT_TRACKER_TIMEOUT_REQUEST_GPS_AGAIN,
	EVT_TRACKER_ALERT_OFF,
}tracker_events_t;
#define SIZE					(	   128		)
#define ENOUGH_DATA_BYTES		(	 	80	 	)
typedef struct _uart_ {
	uint8_t rx[SIZE];
	uint8_t tx[SIZE];
}uart_t;

typedef union {
	struct {
		bool_t new_event:1;
		bool_t alert_on:1;
		bool_t alert_off:1;
		bool_t start_counter_request:1;
		bool_t start_counter_fail:1;
		bool_t start_to_request_nmea_data:1;
	};
	struct {
		uint8_t all_flags:6;
	};

}flags_t;

typedef struct _tracker_fsm_ {
	uint8_t data[SIZE];
	uart_t buff;
	nmea_t nmea;
	uint32_t counter_request;
	uint16_t counter_fail;
	uint16_t counter_request_nmea_data;
	uint16_t *time;
	tracker_states_t state;
	tracker_events_t event;
	flags_t flag;

}tracker_fsm_t;

void tracker_prepare_communication ( tracker_fsm_t *fsm );
void tracker_send_request_to_gps (tracker_fsm_t *fsm );
void tracker_process_data_received_from_gps ( tracker_fsm_t *fsm );
void tracker_set_event ( tracker_fsm_t *fsm, tracker_events_t event );
void tracker_fsm_init ( tracker_fsm_t *fsm, but_fsm_t *but );
void tracker_fsm_run ( tracker_fsm_t *fsm );
void tracker_parse_nmea_data ( tracker_fsm_t *fsm, uint8_t *buff, size_t size );

#endif /* INC_TRACKER_H_ */

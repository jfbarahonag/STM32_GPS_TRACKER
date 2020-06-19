/*
 * tracker.c
 *
 *  Created on: 18/06/2020
 *      Author: juanf
 */
#include "tracker.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void tracker_prepare_communication ( tracker_fsm_t *fsm ) {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	HAL_UART_Transmit(&huart1, (uint8_t *)"ATE0\r", strlen("ATE0\r"), 100); /* turn off echo mode */
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+QGNSSC=1\r", strlen("AT+QGNSSC=1\r"), 100); /* turn on GNSS module */
	fsm->flag.start_counter_request = true;
	fsm->state = STATE_TRACKER_WAIT_FOR_REQUEST_GPS;
	HAL_UART_Receive_IT(&huart1, fsm->buff.rx, ENOUGH_DATA_BYTES);
}

void tracker_send_request_to_gps (tracker_fsm_t *fsm ) {

	fsm->flag.start_counter_request = false;
	fsm->flag.start_to_request_nmea_data = true;

}

void tracker_process_data_received_from_gps ( tracker_fsm_t *fsm ) {
	memcpy(fsm->buff.tx, fsm->data, ENOUGH_DATA_BYTES);
	uint8_t *buff_ptr = fsm->buff.rx;

}

void tracker_set_event ( tracker_fsm_t *fsm, tracker_events_t event ) {
	fsm->event = event;
	fsm->flag.new_event = true;
}

void tracker_fsm_init ( tracker_fsm_t *fsm, but_fsm_t *but ) {
	memset(fsm->buff.rx, '\0', SIZE);
	memset(fsm->buff.tx, '\0', SIZE);
	memset(fsm->data, '\0', SIZE);
	memset(fsm->nmea.all_data, '\0', sizeof(fsm->nmea.all_data));
	fsm->state = STATE_TRACKER_IDLE;
	fsm->event = EVT_TRACKER_NO_EVT;
	fsm->time = &but->time_being_pressed;
	fsm->counter_request_nmea_data = 0;
	fsm->counter_fail = 0;
	fsm->counter_request = 0;
	fsm->flag.all_flags = 0x00;
}

void tracker_fsm_run ( tracker_fsm_t *fsm ) {
	if( fsm->flag.new_event == true ) {
		fsm->flag.new_event = false;

		switch (fsm->state) {

		case STATE_TRACKER_IDLE:
			if (fsm->event == EVT_TRACKER_ALERT_ON) {
				tracker_prepare_communication(fsm);
			}
			break;

		case STATE_TRACKER_WAIT_FOR_REQUEST_GPS:

			if (fsm->event == EVT_TRACKER_TIMEOUT_REQUEST_GPS) {

				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
				tracker_send_request_to_gps(fsm);
				HAL_UART_Receive_IT(&huart1, fsm->buff.rx, ENOUGH_DATA_BYTES);

				fsm->state = STATE_TRACKER_SEND_REQUEST_GPS;
			}
			break;

		case STATE_TRACKER_SEND_REQUEST_GPS:

			if (fsm->event == EVT_TRACKER_GPS_REQUEST_SENT) { /* transition state */

				fsm->state = STATE_TRACKER_WAITING_GPS_RESPONSE;

			}
			break;

		case STATE_TRACKER_WAITING_GPS_RESPONSE:

			if (fsm->event == EVT_TRACKER_GPS_RESPONSE_RECEIVED) {

				tracker_parse_nmea_data(fsm, fsm->data, ENOUGH_DATA_BYTES);
				//tracker_process_data_received_from_gps(fsm);
				HAL_UART_Transmit_IT(&huart2, fsm->nmea.all_data, sizeof(fsm->nmea.all_data));
			}

			break;

		case STATE_TRACKER_RECEIVED_GPS:

			//tracker_parse_nmea_data(fsm->buff.rx, strlen((const char *)(fsm->buff.rx)));
			break;

		case STATE_TRACKER_GSM_SENDING:

			break;

		default: /* ERROR */

			while(1);
			break;
		}
	}
}

void tracker_parse_nmea_data ( tracker_fsm_t *fsm, uint8_t *buff, size_t size ) {
	uint8_t *start, *end;

	start = (uint8_t *)strnstr((const char *)buff, (const char *)"$GNRMC", size);
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);
	/* ----- TIME ----- */
	start = end+1; /* avoid comma of $GNRMC, */
	memcpy(fsm->nmea.gnrmc.time, start, sizeof(fsm->nmea.gnrmc.time));
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);
	/* ----- VALID ----- */
	start = end+1;
	memcpy(fsm->nmea.gnrmc.valid, start, sizeof(fsm->nmea.gnrmc.valid));
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);
	/* ----- LATITUDE ----- */
	start = end+1;
	memcpy(fsm->nmea.gnrmc.latitude, start, sizeof(fsm->nmea.gnrmc.latitude));
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);
	/* ----- N/S ----- */
	start = end+1;
	memcpy(fsm->nmea.gnrmc.N_S, start, sizeof(fsm->nmea.gnrmc.N_S));
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);
	/* ----- LONGITUDE ----- */
	start = end+1;
	memcpy(fsm->nmea.gnrmc.longitude, start, sizeof(fsm->nmea.gnrmc.longitude));
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);
	/* ----- E/W ----- */
	start = end+1;
	memcpy(fsm->nmea.gnrmc.E_W, start, sizeof(fsm->nmea.gnrmc.E_W));
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);
	/* ----- SPEED ----- */
	start = end+1;
	memcpy(fsm->nmea.gnrmc.speed, start, sizeof(fsm->nmea.gnrmc.speed));
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);
	/* ----- COG ----- */
	start = end+1;
	memcpy(fsm->nmea.gnrmc.COG, start, sizeof(fsm->nmea.gnrmc.COG));
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);
	/* ----- DATE ----- */
	start = end+1;
	memcpy(fsm->nmea.gnrmc.date, start, sizeof(fsm->nmea.gnrmc.date));
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size);

	size_t start_size = sizeof("$GNRMC")+1; /* including the comma */
	start += start_size;
	start++;
	end = (uint8_t *)strnstr((const char *)start, (const char *)",", size-start_size);



}



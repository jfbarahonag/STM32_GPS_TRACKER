/*
 * nmea.h
 *
 *  Created on: 18/06/2020
 *      Author: juanf
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_

typedef struct _gnrmc_ {
	uint8_t time[10];
	uint8_t valid[1];
	uint8_t latitude[9];
	uint8_t N_S[1];
	uint8_t longitude[10];
	uint8_t E_W[1];
	uint8_t speed[4];
	uint8_t COG[4];
	uint8_t date[6];
}gnrmc_t;

typedef union _nmea_ {
	struct{
		gnrmc_t gnrmc;
	};
	struct{
		uint8_t all_data[sizeof(gnrmc_t)];
	};
}nmea_t;

#endif /* INC_NMEA_H_ */

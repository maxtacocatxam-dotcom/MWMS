/*
 * GPSFunctions.h
 *
 *  Created on: Apr 7, 2026
 *      Author: Maxta
 */

#ifndef INC_APP_GPS_H_
#define INC_APP_GPS_H_
#include <stdbool.h>
#include <stdint.h>

typedef enum {
	GPS_OK = 0,
	GPS_ERROR = 1
} GPS_Status;



//location data
typedef struct {
    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    bool     validDate, validTime;
    bool     gnssFixOK;
    int32_t  lat;   // deg * 1e-7
    int32_t  lon;   // deg * 1e-7
} GPS_PVT;

GPS_Status gps_init(void);

GPS_Status gps_send(uint8_t msg_class, uint8_t msg_id,
					const uint8_t *payload, uint16_t payload_len);

GPS_Status gps_config_nmea(uint8_t state);

GPS_Status gps_call_location(GPS_PVT *out);

#endif /* INC_APP_GPS_H_ */

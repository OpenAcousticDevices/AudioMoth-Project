/****************************************************************************
 * gpsutilities.h
 * openacousticdevices.info
 * July 2020
 *****************************************************************************/
 
#ifndef __GPSUTILITIES_H
#define __GPSUTILITIES_H

#include "nmeaparser.h"

void GPSUtilities_getTime(NMEA_parserResultRMC_t *result, uint32_t *time);

void GPSUtilities_getLatitude(NMEA_parserResultRMC_t *result, float *latitude);

void GPSUtilities_getLongitude(NMEA_parserResultRMC_t *result, float *longitude);

void GPSUtilities_getFractionalYearInRadians(NMEA_parserResultRMC_t *result, float *gamma);

bool GPSUtilities_calculateSunsetAndSunrise(float gamma, float latitude, float longitude, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes);

#endif /* __GPSUTILITIES_H */

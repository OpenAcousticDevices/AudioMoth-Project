/****************************************************************************
 * sunrise.h
 * openacousticdevices.info
 * July 2020
 *****************************************************************************/

#ifndef __SUNRISE_H
#define __SUNRISE_H

bool Sunrise_calculateFromDate(uint32_t year, uint32_t month, uint32_t day, float latitude, float longitude, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes);

bool Sunrise_calculateFromUnix(uint32_t currentTime, float latitude, float longitude, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes);

#endif /* __SUNRISE_H */

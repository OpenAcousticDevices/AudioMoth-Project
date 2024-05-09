/****************************************************************************
 * sunrise.h
 * openacousticdevices.info
 * July 2020
 *****************************************************************************/

#ifndef __SUNRISE_H
#define __SUNRISE_H

typedef enum {SR_SUN_ABOVE_HORIZON, SR_NORMAL_SOLUTION, SR_SUN_BELOW_HORIZON} SR_solution_t;

typedef enum {SR_DAY_LONGER_THAN_NIGHT, SR_DAY_EQUAL_TO_NIGHT, SR_DAY_SHORTER_THAN_NIGHT} SR_trend_t;

typedef enum {SR_SUNRISE_AND_SUNSET, SR_CIVIL_DAWN_AND_DUSK, SR_NAUTICAL_DAWN_AND_DUSK, SR_ASTRONOMICAL_DAWN_AND_DUSK} SR_event_t;

void Sunrise_calculateFromDate(SR_event_t event, uint32_t year, uint32_t month, uint32_t day, float latitude, float longitude, SR_solution_t *solution, SR_trend_t *trend, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes);

void Sunrise_calculateFromUnix(SR_event_t event, uint32_t currentTime, float latitude, float longitude, SR_solution_t *solution, SR_trend_t *trend, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes);

#endif /* __SUNRISE_H */

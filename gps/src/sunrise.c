/****************************************************************************
 * sunrise.c
 * openacousticdevices.info
 * July 2020
 *****************************************************************************/

#include <math.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#include "sunrise.h"

#define MINUTES_IN_DAY                  1440.0f

#define DAYS_IN_YEAR                    365.0f
#define DAYS_IN_LEAP_YEAR               366.0f

#define HOURS_IN_DAY                    24.0f
#define HOURS_TO_MIDDAY                 12.0f
#define MINUTES_IN_HOUR                 60.0f
#define SECONDS_IN_HOUR                 3600.0f

#define YEAR_OFFSET                     1900

#ifndef M_PI
#define M_PI                            3.14159265358979323846264338328f
#endif

static const float zenithAngles[4] = {90.833f / 180.0f * M_PI, 96.0f / 180.0f * M_PI, 102.0f / 180.0f * M_PI, 108.0f / 180.0f * M_PI};

static const uint32_t daysByMonth[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

static bool isLeapYear(uint32_t year) {

    return (year & 3) == 0 && ((year % 25) != 0 || (year & 15) == 0);

}

static uint32_t dayOfYear(uint32_t year, uint32_t month, uint32_t day) {
    
    uint32_t days = daysByMonth[month - 1] + day - 1;

    if (isLeapYear(year) && month > 2) days += 1;

    return days;

}

static float normalise(float minutes) {

    if (minutes > MINUTES_IN_DAY) minutes -= MINUTES_IN_DAY;

    if (minutes < 0) minutes += MINUTES_IN_DAY;

    return minutes;

}

static void calculateSunsetAndSunrise(SR_event_t event, float gamma, float latitude, float longitude, SR_solution_t *solution, SR_trend_t *trend, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes) {

    /* Convert latitude to radians */

    latitude = latitude / 180.0f * M_PI;

    /* Calculate equation of time, declination and hour angle */

    float zenith = zenithAngles[event];

    float equationOfTime = 229.18f * (0.000075f + 0.001868f * cos(gamma) - 0.032077f * sinf(gamma) - 0.014615f * cosf(2.0f * gamma) - 0.040849f * sinf(2.0f * gamma));
    
    float decl = 0.006918f - 0.399912f * cosf(gamma) + 0.070257f * sinf(gamma) - 0.006758f * cosf(2.0f * gamma) + 0.000907f * sinf(2.0f * gamma) - 0.002697f * cosf(3.0f * gamma) + 0.00148f * sinf(3.0f * gamma);
    
    float argument = cosf(zenith) / cosf(latitude) / cosf(decl) - tanf(latitude) * tanf(decl);

    *solution = argument > 1.0f ? SR_SUN_BELOW_HORIZON : argument < -1.0f ? SR_SUN_ABOVE_HORIZON : SR_NORMAL_SOLUTION;

    *trend = argument < 0.0f ? SR_DAY_LONGER_THAN_NIGHT : SR_DAY_SHORTER_THAN_NIGHT;

    if (argument < -1.0f) argument = -1.0f;

    if (argument > 1.0f) argument = 1.0f;

    float ha = acosf(argument);
    
    /* Calculate sunrise and sunset */

    float sunrise = normalise(MINUTES_IN_DAY / 2.0f - 4.0f * (longitude + ha / M_PI * 180.0f) - equationOfTime);

    float sunset = normalise(MINUTES_IN_DAY / 2.0f - 4.0f * (longitude - ha / M_PI * 180.0f) - equationOfTime);

    *sunriseMinutes = (uint32_t)roundf(sunrise) % (uint32_t)MINUTES_IN_DAY;

    *sunsetMinutes = (uint32_t)roundf(sunset) % (uint32_t)MINUTES_IN_DAY;

    uint32_t offsetSunrise = *sunriseMinutes + (uint32_t)MINUTES_IN_DAY / 2;

    offsetSunrise = offsetSunrise % (uint32_t)MINUTES_IN_DAY;

    if (offsetSunrise == *sunsetMinutes) *trend = SR_DAY_EQUAL_TO_NIGHT;

}

void Sunrise_calculateFromDate(SR_event_t event, uint32_t year, uint32_t month, uint32_t day, float latitude, float longitude, SR_solution_t *solution, SR_trend_t *trend, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes) {

    /* Calculate fractional part of year */
 
    float totalDaysInYear = isLeapYear(year) ? DAYS_IN_LEAP_YEAR : DAYS_IN_YEAR;

    float gamma = 2.0f * M_PI * (float)dayOfYear(year, month, day) / totalDaysInYear;
 
    /* Calculate sunrise and sunset */

    calculateSunsetAndSunrise(event, gamma, latitude, longitude, solution, trend, sunriseMinutes, sunsetMinutes);
    
}

void Sunrise_calculateFromUnix(SR_event_t event, uint32_t currentTime, float latitude, float longitude, SR_solution_t *solution, SR_trend_t *trend, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes) {

    /* Calculate fractional part of year */

    const time_t rawTime = currentTime;

    struct tm *time = gmtime(&rawTime);

    uint32_t year = YEAR_OFFSET + time->tm_year;

    float totalDaysInYear = isLeapYear(year) ? DAYS_IN_LEAP_YEAR : DAYS_IN_YEAR;

    float dayOfYear = (float)time->tm_yday + (time->tm_hour + time->tm_min / MINUTES_IN_HOUR + time->tm_sec / SECONDS_IN_HOUR - HOURS_TO_MIDDAY) / HOURS_IN_DAY;

    if (dayOfYear < 0) dayOfYear += totalDaysInYear;

    if (dayOfYear >= totalDaysInYear) dayOfYear -= totalDaysInYear;

    float gamma = 2.0f * M_PI * dayOfYear / totalDaysInYear;

    /* Calculate sunrise and sunset */

    calculateSunsetAndSunrise(event, gamma, latitude, longitude, solution, trend, sunriseMinutes, sunsetMinutes);

}

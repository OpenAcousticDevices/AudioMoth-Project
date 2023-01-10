/****************************************************************************
 * gpsutilities.c
 * openacousticdevices.info
 * July 2020
 *****************************************************************************/

#include <time.h>
#include <math.h>
#include <stdbool.h>

#include "gpsutilities.h"

#define SECONDS_IN_DAY                  (3600 * 24)
#define MINUTES_IN_DAY                  1440.0f
#define DAYS_FROM_YEAR_0_TO_1970        719162

#ifndef M_PI
#define M_PI                            3.14159265358979323846264338328f
#endif

#define ZENITH                          (90.833f / 180.0f * M_PI)

static const uint32_t daysByMonth[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

static bool isLeapYear(uint32_t year) {
    
    return (year & 3) == 0 && ((year % 25) != 0 || (year & 15) == 0);

}

static uint32_t daysFrom1970(uint32_t year) {
  
    year -= 1;
  
    return 365 * year + (year / 400) - (year / 100) + (year / 4) - DAYS_FROM_YEAR_0_TO_1970;

}

static uint32_t dayOfYear(uint32_t year, uint32_t month, uint32_t day) {
  
    uint32_t days = daysByMonth[month - 1] + day - 1;

    if (isLeapYear(year) && month > 2) days += 1;

    return days;

}

void GPSUtilities_getTime(NMEA_parserResultRMC_t *result, uint32_t *time) {
    
    uint32_t daysSinceEpoch = daysFrom1970(result->year) + dayOfYear(result->year, result->month, result->day);
        
    *time = SECONDS_IN_DAY * daysSinceEpoch + 3600 * result->hours + 60 * result->minutes + result->seconds;
    
}

void GPSUtilities_getLatitude(NMEA_parserResultRMC_t *result, float *latitude) {

    float lat = (float)result->latitudeDegrees + (float)result->latitudeMinutes / 60.0f + (float)result->latitudeTenThousandths / 600000.0f;

    *latitude = result->latitudeDirection == 'N' ? lat : -lat;

}

void GPSUtilities_getLongitude(NMEA_parserResultRMC_t *result, float *longitude) {

    float lon = (float)result->longitudeDegrees + (float)result->longitudeMinutes / 60.0f + (float)result->longitudeTenThousandths / 600000.0f;
    
    *longitude = result->longitudeDirection == 'E' ? lon : -lon;
    
}

void GPSUtilities_getFractionalYearInRadians(NMEA_parserResultRMC_t *result, float *gamma) {

    float totalDaysInYear = isLeapYear(result->year) ? 366.0f : 365.0f;

    float fractionalDaysSoFar = (float)dayOfYear(result->year, result->month, result->day);

    *gamma = 2.0f * M_PI * fractionalDaysSoFar / totalDaysInYear;

}

float normalise(float x) {

    if (x > MINUTES_IN_DAY) x -= MINUTES_IN_DAY;
    
    if (x < 0) x += MINUTES_IN_DAY;

    return x;

}

bool GPSUtilities_calculateSunsetAndSunrise(float gamma, float latitude, float longitude, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes) {

    /* Convert latitude to radians */

    latitude = latitude / 180.0f * M_PI;

    /* Calculate equation of time, declination and hour angle */

    float equationOfTime = 229.18f * (0.000075f + 0.001868f * cosf(gamma) - 0.032077f * sinf(gamma) - 0.014615f * cosf(2.0f * gamma) - 0.040849f * sinf(2.0f * gamma));

    float decl = 0.006918f - 0.399912f * cosf(gamma) + 0.070257f * sinf(gamma) - 0.006758f * cosf(2.0f * gamma) + 0.000907f * sinf(2.0f * gamma) - 0.002697f * cosf(3.0f * gamma) + 0.00148f * sinf(3.0f * gamma);

    float ha = acosf(cosf(ZENITH) / cosf(latitude) / cosf(decl) - tanf(latitude) * tanf(decl));

    if (isnan(ha)) return false;

    /* Calculate sunrise and sunset */

    float sunrise = normalise(MINUTES_IN_DAY / 2.0f - 4.0f * (longitude + ha / M_PI * 180.0f) - equationOfTime);

    float sunset = normalise(MINUTES_IN_DAY / 2.0f - 4.0f * (longitude - ha / M_PI * 180.0f) - equationOfTime);

    *sunriseMinutes = (uint32_t)roundf(sunrise);

    *sunsetMinutes = (uint32_t)roundf(sunset);

    return true;
    
}

/****************************************************************************
 * sunrise.c
 * openacousticdevices.info
 * July 2020
 *****************************************************************************/

#include <math.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#define MINUTES_IN_DAY                  1440.0f

#ifndef M_PI
#define M_PI                            3.14159265358979323846264338328f
#endif

#define ZENITH                          (90.833f / 180.0f * M_PI)

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

static bool calculateSunsetAndSunrise(float gamma, float latitude, float longitude, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes) {

    /* Convert latitude to radians */

    latitude = latitude / 180.0f * M_PI;

    /* Calculate equation of time, declination and hour angle */

    float equationOfTime = 229.18f * (0.000075f + 0.001868f * cos(gamma) - 0.032077f * sinf(gamma) - 0.014615f * cosf(2.0f * gamma) - 0.040849f * sinf(2.0f * gamma));
    
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

bool Sunrise_calculateFromDate(uint32_t year, uint32_t month, uint32_t day, float latitude, float longitude, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes) {

    /* Calculate fractional part of year */
 
    float totalDaysInYear = isLeapYear(year) ? 366.0f : 365.0f;

    float gamma = 2.0f * M_PI * (float)dayOfYear(year, month, day) / totalDaysInYear;
 
    /* Calculate sunrise and sunset */

    return calculateSunsetAndSunrise(gamma, latitude, longitude, sunriseMinutes, sunsetMinutes);
    
}


bool Sunrise_calculateFromUnix(uint32_t currentTime, float latitude, float longitude, uint32_t *sunriseMinutes, uint32_t *sunsetMinutes) {

    /* Calculate fractional part of year */

    const time_t rawTime = currentTime;

    struct tm *time = gmtime(&rawTime);

    uint32_t year = 1900 + time->tm_year;

    float totalDaysInYear = isLeapYear(year) ? 366.0f : 365.0f;

    float gamma = 2.0f * M_PI * (float)time->tm_yday / totalDaysInYear;

    /* Calculate sunrise and sunset */

    return calculateSunsetAndSunrise(gamma, latitude, longitude, sunriseMinutes, sunsetMinutes);

}

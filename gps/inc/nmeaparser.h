/****************************************************************************
 * nmeaparser.h
 * openacousticdevices.info
 * July 2020
 *****************************************************************************/
 
#ifndef __NMEAPARSER_H
#define __NMEAPARSER_H

#include <stdint.h>
#include <stdbool.h>

#define GGA_BUFFER_LENGTH 8
#define MAX_BUFFER_LENGTH 128

typedef enum {NMEA_WAITING, NMEA_PARSING, NMEA_CHARACTER_ERROR, NMEA_CHECKSUM_ERROR, NMEA_SUCCESS} NMEA_parserStatus_t;

#pragma pack(push, 1)

typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint16_t milliseconds;
    uint8_t latitudeDegrees;
    uint8_t latitudeMinutes;
    uint16_t latitudeTenThousandths;
    char latitudeDirection;
    uint8_t longitudeDegrees;
    uint8_t longitudeMinutes;
    uint16_t longitudeTenThousandths;
    char longitudeDirection;
    char positionFixIndicator;
    uint8_t satellitesUsed;
    char horizontalDOP[GGA_BUFFER_LENGTH];
    char altitude[GGA_BUFFER_LENGTH];
    char geoidalSeparation[GGA_BUFFER_LENGTH];
} NMEA_parserResultGGA_t;

typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint16_t milliseconds;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    char status;
    uint8_t latitudeDegrees;
    uint8_t latitudeMinutes;
    uint16_t latitudeTenThousandths;
    char latitudeDirection;
    uint8_t longitudeDegrees;
    uint8_t longitudeMinutes;
    uint16_t longitudeTenThousandths;
    char longitudeDirection;
} NMEA_parserResultRMC_t;

typedef struct {
    uint16_t length;
    char message[MAX_BUFFER_LENGTH];
} NMEA_parserResultDEFAULT_t;

#pragma pack(pop)

NMEA_parserStatus_t NMEAParser_parseGGA(char c, NMEA_parserResultGGA_t *result);

NMEA_parserStatus_t NMEAParser_parseRMC(char c, NMEA_parserResultRMC_t *result);

NMEA_parserStatus_t NMEAParser_parseDEFAULT(char c, NMEA_parserResultDEFAULT_t *result);

#endif /* __NMEAPARSER_H */

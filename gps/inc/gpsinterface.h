/****************************************************************************
 * gpsinterface.h
 * openacousticdevices.info
 * October 2022
 *****************************************************************************/

#ifndef __GPSINTERFACE_H
#define __GPSINTERFACE_H

#include <stdbool.h>

/* Interrupt handlers */

extern void GPSInterface_handlePulsePerSecond(uint32_t counter, uint32_t counterPeriod, uint32_t counterFrequency);

extern void GPSInterface_handleReceivedByte(uint8_t byte);

extern void GPSInterface_handleTick(void);

/* Disable, enable and send functions */

void GPSInterface_enable(uint32_t ticksPerSecond);

void GPSInterface_disableTicks(void);

void GPSInterface_disable(void);

#endif /* __GPSINTERFACE_H */

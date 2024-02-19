/****************************************************************************
 * main.c
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#include <stdbool.h>

#include "audioMoth.h"

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 0, 0};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "Example-Firmware";

/* Required time zone handler */

void AudioMoth_timezoneRequested(int8_t *timezoneHours, int8_t *timezoneMinutes) { }

/* Required interrupt handles */

void AudioMoth_handleSwitchInterrupt() { }
void AudioMoth_handleMicrophoneChangeInterrupt() { }
void AudioMoth_handleMicrophoneInterrupt(int16_t sample) { }
void AudioMoth_handleDirectMemoryAccessInterrupt(bool primaryChannel, int16_t **nextBuffer) { }

/* Required USB message handlers */

void AudioMoth_usbFirmwareVersionRequested(uint8_t **firmwareVersionPtr) {

    *firmwareVersionPtr = firmwareVersion;

}

void AudioMoth_usbFirmwareDescriptionRequested(uint8_t **firmwareDescriptionPtr) {

    *firmwareDescriptionPtr = firmwareDescription;

}

void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size) { }
void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t *receiveBuffer, uint8_t *transmitBuffer, uint32_t size) { }

/* Main function */

int main() {

    /* Initialise device */

    AudioMoth_initialise();

    /* Check the switch position */

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (switchPosition == AM_SWITCH_USB) {

        /* Handle the case that the switch is in USB position. Waits in low energy state until USB disconnected or switch moved  */

        AudioMoth_handleUSB();

    } else {

        /* Flash both LED */

        AudioMoth_setBothLED(true);

        AudioMoth_delay(100);

        AudioMoth_setBothLED(false);

    }

    /* Power down and wake up in one second */

    AudioMoth_powerDownAndWake(1, true);

}
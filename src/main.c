/****************************************************************************
 * main.c
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#include <stdbool.h>

#include "audioMoth.h"

/* Required interrupt handles */

void AudioMoth_handleSwitchInterrupt(void) { };
void AudioMoth_handleMicrophoneInterrupt(int16_t sample) { };
void AudioMoth_handleDirectMemoryAccessInterrupt(bool primaryChannel, int16_t **nextBuffer) { };
void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size) { };
void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t *receiveBuffer, uint8_t *transmitBuffer, uint32_t size) { };

/* Main function */

int main(void) {

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
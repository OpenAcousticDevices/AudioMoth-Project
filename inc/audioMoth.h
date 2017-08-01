/****************************************************************************
 * audioMoth.h
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#define AM_EXTERNAL_SRAM_START_ADDRESS         0x80000000
#define AM_EXTERNAL_SRAM_SIZE_IN_BYTES         (256 * 1024)

#define AM_BACKUP_DOMAIN_START_ADDRESS         0x40081120
#define AM_BACKUP_DOMAIN_SIZE_IN_BYTES         480

#define AM_UNIQUE_ID_START_ADDRESS             0xFE081F0
#define AM_UNIQUE_ID_SIZE_IN_BYTES             8

/* Switch and battery state enumerations */

typedef enum {AM_SWITCH_CUSTOM, AM_SWITCH_DEFAULT, AM_SWITCH_USB, AM_SWITCH_NONE} AM_switchPosition_t;

typedef enum {AM_HFRCO_1MHZ, AM_HFRCO_7MHZ, AM_HFRCO_11MHZ, AM_HFRCO_14MHZ, AM_HFRCO_21MHZ, AM_HFRCO_28MHZ, AM_HFXO} AM_clockFrequency_t;

typedef enum {AM_BATTERY_LOW, AM_BATTERY_3V6, AM_BATTERY_3V7, AM_BATTERY_3V8, AM_BATTERY_3V9, AM_BATTERY_4V0, AM_BATTERY_4V1, AM_BATTERY_4V2, AM_BATTERY_4V3, AM_BATTERY_4V4, AM_BATTERY_4V5, AM_BATTERY_4V6, AM_BATTERY_4V7, AM_BATTERY_4V8, AM_BATTERY_4V9, AM_BATTERY_FULL } AM_batteryState_t;

/* Interrupt handlers */

extern void AudioMoth_handleSwitchInterrupt(void);
extern void AudioMoth_handleMicrophoneInterrupt(int16_t sample);
extern void AudioMoth_handleDirectMemoryAccessInterrupt(bool primaryChannel, int16_t **nextBuffer);

/* USB message handlers */

extern void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size);
extern void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t *receiveBuffer, uint8_t *transmitBuffer, uint32_t size);

/* Initialise device */

void AudioMoth_initialise(void);
bool AudioMoth_isInitialPowerUp(void);

/* Clock control */

void AudioMoth_enableHFXO(void);
void AudioMoth_selectHFXO(void);
void AudioMoth_disableHFXO(void);

void AudioMoth_enableHFRCO(AM_clockFrequency_t frequency);
void AudioMoth_selectHFRCO(void);
void AudioMoth_disableHFRCO(void);

void AudioMoth_calibrateHFRCO(uint32_t frequency);

uint32_t AudioMoth_getClockFrequency(AM_clockFrequency_t frequency);

/* External SRAM control */

void AudioMoth_enableExternalSRAM(void);
void AudioMoth_disableExternalSRAM(void);

/* Microphone samples */

void AudioMoth_startMicrophoneSamples(void);
void AudioMoth_initialiseMicrophoneInterupts(void);
void AudioMoth_initialiseDirectMemoryAccess(int16_t *primaryBuffer, int16_t *secondaryBuffer, uint16_t numberOfSamples);

uint32_t AudioMoth_calculateSampleRate(uint32_t frequency, uint32_t clockDivider, uint32_t acquisitionCycles, uint32_t oversampleRate);

void AudioMoth_enableMicrophone(uint32_t gain, uint32_t clockDivider, uint32_t acquisitionCycles, uint32_t oversampleRate);
void AudioMoth_disableMicrophone(void);

/* USB */

void AudioMoth_handleUSB(void);

/* Backup domain */

uint32_t AudioMoth_retreiveFromBackupDomain(uint32_t register);
void AudioMoth_storeInBackupDomain(uint32_t register, uint32_t value);

/* Time */

uint32_t AudioMoth_getTime(void);
bool AudioMoth_hasTimeBeenSet(void);
void AudioMoth_setTime(uint32_t time);

/* Watch dog timer */

void AudioMoth_startWatchdog(void);
void AudioMoth_stopWatchdog(void);

void AudioMoth_feedWatchdog(void);
bool AudioMoth_hasWatchdogResetOccured(void);

/* Battery state monitoring */

AM_batteryState_t AudioMoth_getBatteryState();

/* Switch position monitoring */

AM_switchPosition_t AudioMoth_getSwitchPosition();

/* Busy delay */

void AudioMoth_delay(uint16_t milliseconds);

/* Sleep and power down */

void AudioMoth_sleep();
void AudioMoth_powerDown();
void AudioMoth_powerDownAndWake(uint32_t seconds, bool synchronised);

/* LED control */

void AudioMoth_setRedLED(bool state);
void AudioMoth_setBothLED(bool state);
void AudioMoth_setGreenLED(bool state);

/* File system */

bool AudioMoth_enableFileSystem();
void AudioMoth_disableFileSystem();

bool AudioMoth_openFile(char *filename);
bool AudioMoth_appendFile(char *filename);

bool AudioMoth_seekInFile(uint32_t position);
bool AudioMoth_writeToFile(void *bytes, uint16_t bytesToWrite);

bool AudioMoth_renameFile(char *originalFilename, char *newFilename);

bool AudioMoth_closeFile();

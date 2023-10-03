/****************************************************************************
 * audiomoth.h
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#ifndef __AUDIOMOTH_H
#define __AUDIOMOTH_H

#include <stdint.h>
#include <stdbool.h>

/* AudioMoth constants */

#define AM_FIRMWARE_VERSION_LENGTH             3
#define AM_FIRMWARE_DESCRIPTION_LENGTH         32

#define AM_EXTERNAL_SRAM_START_ADDRESS         0x80000000
#define AM_EXTERNAL_SRAM_SIZE_IN_BYTES         (256 * 1024)

#define AM_BACKUP_DOMAIN_START_ADDRESS         0x40081120
#define AM_BACKUP_DOMAIN_SIZE_IN_REGISTERS     120
#define AM_BACKUP_DOMAIN_SIZE_IN_BYTES         480

#define AM_FLASH_USER_DATA_ADDRESS             0xFE00000
#define AM_FLASH_USER_SIZE_IN_BYTES            2048

#define AM_UNIQUE_ID_START_ADDRESS             0xFE081F0
#define AM_UNIQUE_ID_SIZE_IN_BYTES             8

#define AM_BATTERY_STATE_OFFSET                3500
#define AM_EXT_BAT_STATE_OFFSET                2400
#define AM_BATTERY_STATE_INCREMENT             100

/* Gain, SD card speed, switch, frequency and battery state enumerations */

typedef enum {AM_LOW_GAIN_RANGE, AM_NORMAL_GAIN_RANGE} AM_gainRange_t;

typedef enum {AM_SD_CARD_NORMAL_SPEED, AM_SD_CARD_HIGH_SPEED} AM_sdCardSpeed_t;

typedef enum {AM_HF_CLK_DIV1, AM_HF_CLK_DIV2, AM_HF_CLK_DIV4} AM_highFrequencyClockDivider_t;

typedef enum {AM_SWITCH_CUSTOM, AM_SWITCH_DEFAULT, AM_SWITCH_USB, AM_SWITCH_NONE} AM_switchPosition_t;

typedef enum {AM_GAIN_LOW, AM_GAIN_LOW_MEDIUM, AM_GAIN_MEDIUM, AM_GAIN_MEDIUM_HIGH, AM_GAIN_HIGH} AM_gainSetting_t;

typedef enum {AM_HFRCO_1MHZ, AM_HFRCO_7MHZ, AM_HFRCO_11MHZ, AM_HFRCO_14MHZ, AM_HFRCO_21MHZ, AM_HFRCO_28MHZ} AM_clockFrequency_t;

typedef enum {AM_BATTERY_LOW, AM_BATTERY_3V6, AM_BATTERY_3V7, AM_BATTERY_3V8, AM_BATTERY_3V9, AM_BATTERY_4V0, AM_BATTERY_4V1, AM_BATTERY_4V2, \
              AM_BATTERY_4V3, AM_BATTERY_4V4, AM_BATTERY_4V5, AM_BATTERY_4V6, AM_BATTERY_4V7, AM_BATTERY_4V8, AM_BATTERY_4V9, AM_BATTERY_FULL } AM_batteryState_t;

typedef enum {AM_EXT_BAT_LOW, AM_EXT_BAT_2V5, AM_EXT_BAT_2V6, AM_EXT_BAT_2V7, AM_EXT_BAT_2V8, AM_EXT_BAT_2V9, AM_EXT_BAT_3V0, AM_EXT_BAT_3V1, \
              AM_EXT_BAT_3V2, AM_EXT_BAT_3V3, AM_EXT_BAT_3V4, AM_EXT_BAT_3V5, AM_EXT_BAT_3V6, AM_EXT_BAT_3V7, AM_EXT_BAT_3V8, AM_EXT_BAT_3V9, \
              AM_EXT_BAT_4V0, AM_EXT_BAT_4V1, AM_EXT_BAT_4V2, AM_EXT_BAT_4V3, AM_EXT_BAT_4V4, AM_EXT_BAT_4V5, AM_EXT_BAT_4V6, AM_EXT_BAT_4V7, \
              AM_EXT_BAT_4V8, AM_EXT_BAT_4V9, AM_EXT_BAT_FULL} AM_extendedBatteryState_t;

/* Time zone handler */

extern void AudioMoth_timezoneRequested(int8_t *timezoneHours, int8_t *timezoneMinutes);

/* Interrupt handlers */

extern void AudioMoth_handleSwitchInterrupt(void);
extern void AudioMoth_handleMicrophoneChangeInterrupt(void);
extern void AudioMoth_handleMicrophoneInterrupt(int16_t sample);
extern void AudioMoth_handleDirectMemoryAccessInterrupt(bool isPrimaryBuffer, int16_t **nextBuffer);

/* USB message handlers */

extern void AudioMoth_usbFirmwareVersionRequested(uint8_t **firmwareVersionPtr);
extern void AudioMoth_usbFirmwareDescriptionRequested(uint8_t **firmwareDescriptionPtr);
extern void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size);
extern void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t *receiveBuffer, uint8_t *transmitBuffer, uint32_t size);

/* Initialise device */

void AudioMoth_initialise(void);
bool AudioMoth_isInitialPowerUp(void);

/* Device status */

bool AudioMoth_hasInvertedOutput(void);

/* Clock control */

void AudioMoth_enableHFXO(void);
void AudioMoth_selectHFXO(void);
void AudioMoth_disableHFXO(void);

void AudioMoth_enableHFRCO(AM_clockFrequency_t frequency);
void AudioMoth_selectHFRCO(void);
void AudioMoth_disableHFRCO(void);

uint32_t AudioMoth_getClockFrequency();

void AudioMoth_setClockDivider(AM_highFrequencyClockDivider_t divider);
AM_highFrequencyClockDivider_t AudioMoth_getClockDivider(void);

/* External SRAM control */

bool AudioMoth_enableExternalSRAM(void);
void AudioMoth_disableExternalSRAM(void);

/* Microphone samples */

void AudioMoth_startMicrophoneSamples(uint32_t sampleRate);

void AudioMoth_initialiseMicrophoneInterrupts(void);
void AudioMoth_initialiseDirectMemoryAccess(int16_t *primaryBuffer, int16_t *secondaryBuffer, uint16_t numberOfSamples);

bool AudioMoth_enableMicrophone(AM_gainRange_t gainRange, AM_gainSetting_t gainSetting, uint32_t clockDivider, uint32_t acquisitionCycles, uint32_t oversampleRate);
void AudioMoth_disableMicrophone(void);

/* USB */

void AudioMoth_handleUSB(void);

/* Backup domain */

uint32_t AudioMoth_retreiveFromBackupDomain(uint32_t number);
void AudioMoth_storeInBackupDomain(uint32_t number, uint32_t value);

/* Flash user data page */

bool AudioMoth_writeToFlashUserDataPage(uint8_t *data, uint32_t length);

/* Time */

bool AudioMoth_hasTimeBeenSet(void);
void AudioMoth_setTime(uint32_t time, uint32_t milliseconds);
void AudioMoth_getTime(uint32_t *time, uint32_t *milliseconds);

/* Watch dog timer */

void AudioMoth_startWatchdog(void);
void AudioMoth_stopWatchdog(void);

void AudioMoth_feedWatchdog(void);
bool AudioMoth_hasWatchdogResetOccurred(void);

/* Real time clock */

void AudioMoth_startRealTimeClock(uint32_t seconds);
void AudioMoth_startRealTimeClockMilliseconds(uint32_t milliseconds);
void AudioMoth_stopRealTimeClock(void);

void AudioMoth_checkAndHandleTimeOverflow(void);

/* Supply voltage monitoring */

void AudioMoth_enableSupplyMonitor(void);
void AudioMoth_disableSupplyMonitor(void);

void AudioMoth_setSupplyMonitorThreshold(uint32_t supplyVoltage);
bool AudioMoth_isSupplyAboveThreshold(void);

/* Battery state monitoring */

void AudioMoth_enableBatteryMonitor(void);
void AudioMoth_disableBatteryMonitor(void);

void AudioMoth_setBatteryMonitorThreshold(uint32_t batteryVoltage, uint32_t supplyVoltage);
bool AudioMoth_isBatteryAboveThreshold(void);

/* Supply voltage and battery voltage / state reporting */

uint32_t AudioMoth_getSupplyVoltage(void);

AM_batteryState_t AudioMoth_getBatteryState(uint32_t supplyVoltage);

AM_extendedBatteryState_t AudioMoth_getExtendedBatteryState(uint32_t supplyVoltage);

/* Temperature monitoring */

void AudioMoth_enableTemperature(void);
void AudioMoth_disableTemperature(void);

int32_t AudioMoth_getTemperature(void);

/* Switch position monitoring */

AM_switchPosition_t AudioMoth_getSwitchPosition(void);

/* Busy delay */

void AudioMoth_delay(uint32_t milliseconds);

/* Sleep and power down */

void AudioMoth_sleep(void);
void AudioMoth_deepSleep(void);
void AudioMoth_powerDown(void);
void AudioMoth_powerDownAndWakeMilliseconds(uint32_t millisecond);
void AudioMoth_powerDownAndWake(uint32_t seconds, bool synchronised);

/* LED control */

void AudioMoth_setRedLED(bool state);
void AudioMoth_setBothLED(bool state);
void AudioMoth_setGreenLED(bool state);

/* File system */

bool AudioMoth_enableFileSystem(AM_sdCardSpeed_t speed);
void AudioMoth_disableFileSystem(void);

bool AudioMoth_doesFileExist(char *filename);

bool AudioMoth_openFile(char *filename);
bool AudioMoth_openFileToRead(char *filename);
bool AudioMoth_readFile(char *buffer, uint32_t bufferSize);
bool AudioMoth_appendFile(char *filename);

bool AudioMoth_seekInFile(uint32_t position);
bool AudioMoth_writeToFile(void *bytes, uint16_t bytesToWrite);

bool AudioMoth_renameFile(char *originalFilename, char *newFilename);

bool AudioMoth_doesDirectoryExist(char *folderName);
bool AudioMoth_makeDirectory(char *folderName);

bool AudioMoth_syncFile(void);
bool AudioMoth_closeFile(void);

/* Debugging */

void AudioMoth_setupSWOForPrint(void);

#endif /* __AUDIOMOTH_H */

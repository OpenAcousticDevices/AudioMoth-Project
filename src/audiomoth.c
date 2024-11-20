/****************************************************************************
 * audiomoth.c
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "em_adc.h"
#include "em_cmu.h"
#include "em_dma.h"
#include "em_emu.h"
#include "em_ebi.h"
#include "em_msc.h"
#include "em_prs.h"
#include "em_rmu.h"
#include "em_rtc.h"
#include "em_usb.h"
#include "em_acmp.h"
#include "em_chip.h"
#include "em_core.h"
#include "em_gpio.h"
#include "em_vcmp.h"
#include "em_wdog.h"
#include "em_burtc.h"
#include "em_opamp.h"
#include "em_timer.h"
#include "em_usart.h"

#include "ff.h"
#include "diskio.h"
#include "dmactrl.h"
#include "microsd.h"

#include "pinouts.h"
#include "usbconfig.h"
#include "usbcallbacks.h"
#include "usbdescriptors.h"

#include "audioMoth.h"

/* Time constants */

#define AM_LFXO_LFRCO_TICKS_PER_SECOND            32768
#define AM_BURTC_TICKS_PER_SECOND                 1024
#define AM_MINIMUM_POWER_DOWN_TIME                64

#define MILLISECONDS_IN_SECOND                    1000
#define SECONDS_IN_MINUTE                         60

/* USB EM2 wake constant */

#define AM_USB_EM2_RTC_WAKEUP_INTERVAL            10

/* Comparator limit constants */

#define MINIMIMUM_COMPARATOR_LEVEL                0
#define MAXIMIMUM_COMPARATOR_LEVEL                63

/* Supply monitor constants */

#define MINIMIMUM_SUPPLY_MONITOR_LEVEL            0
#define MAXIMIMUM_SUPPLY_MONITOR_LEVEL            63

#define VCMP_VOLTAGE_INCREMENT                    34
#define VCMP_VOLTAGE_OFFSET                       1667

/* Battery monitor constants */

#define BATTERY_MONITOR_DIVIDER                   2

#define MINIMIMUM_BATTERY_MONITOR_VOLTAGE         2450
#define MAXIMIMUM_BATTERY_MONITOR_VOLTAGE         4950

/* Temperature constants */

#define MILLIDEGREES_IN_DEGREE                    1000
#define TEMPERATURE_GRADIENT                      63
#define GRADIENT_MULTIPLIER                       10

/*  Define RTC backup register constants */

#define AM_BURTC_TIME_OFFSET_LOW                  0
#define AM_BURTC_TIME_OFFSET_HIGH                 1
#define AM_BURTC_CLOCK_SET_FLAG                   2
#define AM_BURTC_WATCH_DOG_FLAG                   3
#define AM_BURTC_INITIAL_POWER_UP_FLAG            4
#define AM_BURTC_HARDWARE_VERSION                 5

#define AM_BURTC_CANARY_VALUE                     0x11223344

#define AM_BURTC_TOTAL_REGISTERS                  128
#define AM_BURTC_RESERVED_REGISTERS               8

/* USB message types */

#define AM_USB_BUFFERSIZE                         64

#define AM_USB_MSG_TYPE_GET_TIME                  0x01
#define AM_USB_MSG_TYPE_SET_TIME                  0x02
#define AM_USB_MSG_TYPE_GET_UID                   0x03
#define AM_USB_MSG_TYPE_GET_BATTERY               0x04
#define AM_USB_MSG_TYPE_GET_APP_PACKET            0x05
#define AM_USB_MSG_TYPE_SET_APP_PACKET            0x06
#define AM_USB_MSG_TYPE_GET_FIRMWARE_VERSION      0x07
#define AM_USB_MSG_TYPE_GET_FIRMWARE_DESCRIPTION  0x08
#define AM_USB_MSG_TYPE_QUERY_SERIAL_BOOTLOADER   0x09
#define AM_USB_MSG_TYPE_ENTER_SERIAL_BOOTLOADER   0x0A
#define AM_USB_MSG_TYPE_QUERY_USBHID_BOOTLOADER   0x0B
#define AM_USB_MSG_TYPE_ENTER_USBHID_BOOTLOADER   0x0C

/* USB HID bootloader commands */

#define AM_BOOTLOADER_GET_VERSION                 0x01
#define AM_BOOTLOADER_INITIALISE_SRAM             0x02
#define AM_BOOTLOADER_CLEAR_USER_PAGE             0x03
#define AM_BOOTLOADER_SET_SRAM_FIRMWARE_PACKET    0x04
#define AM_BOOTLOADER_CALC_SRAM_FIRMWARE_CRC      0x05
#define AM_BOOTLOADER_CALC_FLASH_FIRMWARE_CRC     0x06
#define AM_BOOTLOADER_GET_FIRMWARE_CRC            0x07
#define AM_BOOTLOADER_FLASH_FIRMWARE              0x08

/* USB HID bootloader constants */

#define FIRMWARE_CRC_POLY                         0x1021

#define AM_FIRMWARE_START_ADDRESS                 (16 * 1024)
#define AM_FIRMWARE_PAGE_SIZE                     (2 * 1024)
#define AM_FIRMWARE_TOTAL_SIZE                    ((256 * 1024) - AM_FIRMWARE_START_ADDRESS)

/* USB blink constant */

#define USB_SET_TIME_BLINK                        200

/* Define WebUSB constants */

#define USB_BOS_DESCRIPTOR                        0x0F

#define WEB_USB_REQUEST_TYPE                      0xC0

#define WEB_USB_URL_REQUEST                       0x01
#define WEB_USB_URL_INDEX                         0x0002

#define WEB_USB_MSFT_REQUEST                      0x02
#define WEB_USB_MSFT_INDEX                        0x0007

/* Useful macros */

#define MIN(a, b)                                 ((a) < (b) ? (a) : (b))

#define MAX(a, b)                                 ((a) > (b) ? (a) : (b))

#define ROUNDED_DIV(a, b)                         (((a) + ((b)/2)) / (b))

/* Hardware type enumeration */

typedef enum {AM_VERSION_1, AM_VERSION_2, AM_VERSION_3, AM_VERSION_4} AM_hardwareVersion_t;

/* USB HID bootloader firmware packet structure */

#pragma pack(push, 1)

typedef struct {
    uint32_t offset;
    uint8_t length;
} usbMessageSetFirmwarePacket_t;

#pragma pack(pop)

/* USB buffers */

STATIC_UBUF(receiveBuffer, 2 * AM_USB_BUFFERSIZE);
STATIC_UBUF(transmitBuffer, 2 * AM_USB_BUFFERSIZE);

/* SD card variables */

static FATFS fatfs;
static FIL file;
static UINT bw;

/* DMA variables */

static DMA_CB_TypeDef cb;
static uint16_t numberOfSamplesPerTransfer;

/* USB loop counter */

static volatile uint32_t usbLoopCounter;

/* Delay timer variable */

static volatile bool delayTimmerRunning;

/* USB bootloader variables */

static volatile uint16_t currentCRC;

static volatile uint8_t* firmwareStartAddress;

static volatile bool shouldCalculateCRC;

static volatile bool completedCalculateCRC;

static volatile bool enterSerialBootloader;

static volatile bool shouldFlashFirmware;

/* Function prototypes */

static void setupGPIO(void);
static void enableEBI(void);
static void disableEBI(void);
static void setupBackupRTC(bool useLFXO);
static void setupBackupDomain(bool useLFXO);
static void setupWatchdogTimer(void);
static void handleTimeOverflow(void);
static void setupOpAmp(AM_gainRange_t gainRain, AM_gainSetting_t gain);
static AM_hardwareVersion_t senseHardwareVersion(void);
static void enablePrsTimer(uint32_t samplerate);
static void setupADC(uint32_t clockDivider, uint32_t acquisitionCycles, uint32_t oversampleRate);

/* Function to initialise the main components */

void AudioMoth_initialise() {

    /* Initialise chip */

    CHIP_Init();

    /* Turn on the FPU */

    SystemInit();

    /* Enable high frequency HFXO clock */

    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_1);

    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

    /* Enable clock to GPIO and low energy modules */

    CMU_ClockEnable(cmuClock_GPIO, true);

    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Store the cause of the last reset, and clear the reset cause register */

    uint32_t resetCause = RMU_ResetCauseGet();

    RMU_ResetCauseClear();

    /* If this was not a regular reset from EM4 then start LFXO and setup the backup domain */

    if (!(resetCause & RMU_RSTCAUSE_EM4WURST)) {

        /* Sense the hardware version */

        AM_hardwareVersion_t hardwareVersion = senseHardwareVersion();

        /* Start the appropriate low frequency oscillator */ 

        if (hardwareVersion < AM_VERSION_4) {

            /* Disable the LFRCO */

            CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);

            /* Disable the LFXO */

            CMU_OscillatorEnable(cmuOsc_LFXO, false, false);

            GPIO_PinModeSet(LFXO_DETECT_GPIOPORT, LFXO_DETECT, gpioModePushPull, 0);

            AudioMoth_delay(100);  

            /* Enable LFXO sense */

            GPIO_PinModeSet(LFXO_DETECT_GPIOPORT, LFXO_DETECT, gpioModePushPull, 1);

            AudioMoth_delay(10);  

            /* Test for presence of crystal */   

            bool crystal = GPIO_PinInGet(LFXO_DETECT_GPIOPORT, LFXO_DETECT);

            /* Disable LFXO sense */

            GPIO_PinModeSet(LFXO_DETECT_GPIOPORT, LFXO_DETECT, gpioModeDisabled, 0);

            /* Start the LFXO */

            if (!crystal) CMU->CTRL |= CMU_CTRL_LFXOMODE_DIGEXTCLK;

            CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

        } else {

            /* Disable the LFXO */

            CMU_OscillatorEnable(cmuOsc_LFXO, false, false);

            /* Start the LFRCO */

            CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

            /* Delay to match LFXO startup time */

            AudioMoth_delay(1000);

        }

        /* Setup backup domain for EM4 */

        setupBackupDomain(hardwareVersion < AM_VERSION_4);

        /* Setup backup RTC */

        setupBackupRTC(hardwareVersion < AM_VERSION_4);

        /* Set the hardware version */

        BURTC_RetRegSet(AM_BURTC_HARDWARE_VERSION, hardwareVersion);

        /* Clear the time set flag and the counter */

        BURTC_RetRegSet(AM_BURTC_CLOCK_SET_FLAG, 0);

        BURTC_RetRegSet(AM_BURTC_TIME_OFFSET_LOW, 0);

        BURTC_RetRegSet(AM_BURTC_TIME_OFFSET_HIGH, 0);

        /* Set the initial power up flag */

        BURTC_RetRegSet(AM_BURTC_INITIAL_POWER_UP_FLAG,  AM_BURTC_CANARY_VALUE);

    }

    /* If this was a regular reset from EM4 check for BURTC overflow and clear flag */

    if (resetCause & RMU_RSTCAUSE_EM4WURST) {

        /* Handle overflow of the BURTC counter if overflow flag has been set */

        AudioMoth_checkAndHandleTimeOverflow();

        /* Clear the initial power up flag */

        BURTC_RetRegSet(AM_BURTC_INITIAL_POWER_UP_FLAG,  0);

    }

    /* If this was a watch dog timer reset then record that this occurred */

    if (resetCause & RMU_RSTCAUSE_WDOGRST) {

        BURTC_RetRegSet(AM_BURTC_WATCH_DOG_FLAG, AM_BURTC_CANARY_VALUE);

    } else {

        BURTC_RetRegSet(AM_BURTC_WATCH_DOG_FLAG, 0);

    }

    /* Put GPIO pins in correct state */

    setupGPIO();

    /* Enable interrupt on USB switch position to wake from EM2 */

    GPIO_PinModeSet(SWITCH_1_GPIOPORT, SWITCH_1_SENSE, gpioModeInput, 0);

    GPIO_PinModeSet(SWITCH_2_GPIOPORT, SWITCH_2_SENSE, gpioModeInput, 0);

    GPIO_IntConfig(SWITCH_1_GPIOPORT, SWITCH_1_SENSE, true, true, true);

    GPIO_IntConfig(SWITCH_2_GPIOPORT, SWITCH_2_SENSE, true, true, true);

    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);

    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

    /* Start the watch dog timer */

    setupWatchdogTimer();

    WDOG_Enable(true);

}

bool AudioMoth_isInitialPowerUp(void) {

    return BURTC_RetRegGet(AM_BURTC_INITIAL_POWER_UP_FLAG) == AM_BURTC_CANARY_VALUE;

}

/* Device status */

bool AudioMoth_hasInvertedOutput(void) {

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    return hardwareVersion >= AM_VERSION_4;

}

/* Clock control */

void AudioMoth_enableHFXO(void) {

    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

}

void AudioMoth_selectHFXO(void) {

    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

}

void AudioMoth_disableHFXO(void) {

    CMU_OscillatorEnable(cmuOsc_HFXO, false, false);

}

void AudioMoth_enableHFRCO(AM_clockFrequency_t frequency) {

    CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);

    CMU_HFRCOBandSet(frequency);

}

void AudioMoth_selectHFRCO() {

    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

}

void AudioMoth_disableHFRCO(void) {

    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

}

uint32_t AudioMoth_getClockFrequency() {

    return CMU_ClockFreqGet(cmuClock_HF);

}

void AudioMoth_setClockDivider(AM_highFrequencyClockDivider_t divider) {

    CMU_ClkDiv_TypeDef clockDivider = divider == AM_HF_CLK_DIV4 ? cmuClkDiv_4 : divider == AM_HF_CLK_DIV2 ? cmuClkDiv_2 : cmuClkDiv_1;

    CMU_ClockDivSet(cmuClock_HF, clockDivider);

}

AM_highFrequencyClockDivider_t AudioMoth_getClockDivider() {

    CMU_ClkDiv_TypeDef clockDivider = CMU_ClockDivGet(cmuClock_HF);

    AM_highFrequencyClockDivider_t divider = clockDivider == cmuClkDiv_4 ? AM_HF_CLK_DIV4 : clockDivider == cmuClkDiv_2 ? AM_HF_CLK_DIV2 : AM_HF_CLK_DIV1;

    return divider;

}

/* Interrupt handler for RTC, switch change events, microphone samples, timer overflow and DMA transfers */

void RTC_IRQHandler(void) {

    /* Get the interrupt mask */

    uint32_t interruptMask = RTC_IntGet();

    /* Clear the interrupt  */

    RTC_IntClear(interruptMask);

    /* Handle the interrupt */

    if (interruptMask & RTC_IFC_COMP0) WDOG_Feed();

}

void GPIO_EVEN_IRQHandler(void) {

    /* Get the interrupt mask */

    uint32_t interruptMask = GPIO_IntGet();

    /* Clear the interrupt */

    GPIO_IntClear(interruptMask);

    /* Handle the interrupt */

    if (interruptMask & ((1 << SWITCH_1_SENSE) | (1 << SWITCH_2_SENSE))) AudioMoth_handleSwitchInterrupt();

    if (interruptMask & ((1 << JCK_DETECT) | (1 << JCK_DETECT_ALT))) AudioMoth_handleMicrophoneChangeInterrupt();

}

void ADC0_IRQHandler(void) {

    /* Get the interrupt mask */

    uint32_t interruptMask = ADC_IntGet(ADC0);
    
    /* Clear the interrupt */

    ADC_IntClear(ADC0, interruptMask);

    /* Handle the interrupt */

    if (interruptMask & ADC_IF_SINGLE) {

        /* Send the sample to the interrupt handler */

        int16_t sample = ADC_DataSingleGet(ADC0);

        AudioMoth_handleMicrophoneInterrupt(sample);

        /* Feed the watch dog timer */

        WDOG_Feed();

    }

}

void TIMER1_IRQHandler(void) {

    /* Get the interrupt mask */

    uint32_t interruptMask = TIMER_IntGet(TIMER1);

    /* Clear the interrupt */

    TIMER_IntClear(TIMER1, interruptMask);

    /* Handle the interrupt */

    if (interruptMask & TIMER_IF_OF) delayTimmerRunning = false;

}

static void transferComplete(unsigned int channel, bool isPrimaryBuffer, void *user) {

    int16_t *nextBuffer = NULL;

    AudioMoth_handleDirectMemoryAccessInterrupt(isPrimaryBuffer, &nextBuffer);

    /* Re-activate the DMA */

    DMA_RefreshPingPong(channel,
        isPrimaryBuffer,
        false,
        (void*)nextBuffer,
        NULL,
        numberOfSamplesPerTransfer - 1,
        false);

    /* Feed the watch dog timer */

    WDOG_Feed();

}

/* Set up backup domain */

static void setupBackupDomain(bool useLFXO) {

    /* Initialise GPIO, BURTC and EM4 registers */

    EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;

    em4Init.vreg = true;
    em4Init.lockConfig = true;
    em4Init.buRtcWakeup = true;
    em4Init.osc = useLFXO ? emuEM4Osc_LFXO : emuEM4Osc_LFRCO;

    /* Unlock configuration */

    EMU_EM4Lock(false);

    EMU_EM4Init(&em4Init);

    /* Enable access to BURTC registers */

    RMU_ResetControl(rmuResetBU, false);

    /* Lock configuration */

    EMU_EM4Lock(true);

}

/* Configure BURTC */

static void setupBackupRTC(bool useLFXO) {

    /* Set up BURTC to count and wake from EM4 */

    BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;

    burtcInit.mode = burtcModeEM4;
    burtcInit.clkSel = useLFXO ? burtcClkSelLFXO : burtcClkSelLFRCO;
    burtcInit.clkDiv = burtcClkDiv_32;
    burtcInit.timeStamp = false;
    burtcInit.compare0Top = false;
    burtcInit.enable = false;
    burtcInit.lowPowerMode = burtcLPDisable;

    BURTC_Init(&burtcInit);

    /* Enable interrupt on counter overflow */

    BURTC_IntClear(BURTC_IF_OF);

    BURTC_IntEnable(BURTC_IF_OF);

    /* Enable the timer */

    BURTC_Enable(true);

}

/* Functions to initialise the microphone */

void AudioMoth_startMicrophoneSamples(uint32_t sampleRate) {

	enablePrsTimer(sampleRate);

    /* Start the ADC samples */

    ADC_Start(ADC0, adcStartSingle);

}

void AudioMoth_initialiseMicrophoneInterrupts(void) {

    /* Enable ADC interrupt vector in NVIC */

    ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

    NVIC_ClearPendingIRQ(ADC0_IRQn);

    NVIC_EnableIRQ(ADC0_IRQn);

}

void AudioMoth_initialiseDirectMemoryAccess(int16_t *primaryBuffer, int16_t *secondaryBuffer, uint16_t numberOfSamples) {

    numberOfSamplesPerTransfer = numberOfSamples;

    if (numberOfSamplesPerTransfer > 1024) numberOfSamplesPerTransfer = 1024;

    /* Start the clock */

    CMU_ClockEnable(cmuClock_DMA, true);

    /* Initialise the DMA structure */

    DMA_Init_TypeDef dmaInit;

    dmaInit.hprot = 0;
    dmaInit.controlBlock = dmaControlBlock;

    DMA_Init(&dmaInit);

    /* Setting up call-back function */

    cb.cbFunc = transferComplete;
    cb.userPtr = NULL;

    /* Setting up channel */

    DMA_CfgChannel_TypeDef chnlCfg;

    chnlCfg.highPri = false;
    chnlCfg.enableInt = true;
    chnlCfg.select = DMAREQ_ADC0_SINGLE;
    chnlCfg.cb = &cb;

    DMA_CfgChannel(0, &chnlCfg);

    /* Setting up channel descriptor */

    DMA_CfgDescr_TypeDef descrCfg;

    descrCfg.dstInc = dmaDataInc2;
    descrCfg.srcInc = dmaDataIncNone;
    descrCfg.size = dmaDataSize2;
    descrCfg.arbRate = dmaArbitrate1;
    descrCfg.hprot = 0;

    /* Set up both the primary and the secondary transfers */

    DMA_CfgDescr(0, true, &descrCfg);
    DMA_CfgDescr(0, false, &descrCfg);

    /* Set up the first transfer */
    DMA_ActivatePingPong(0,
        false,
        (void*)primaryBuffer,
        (void*)&(ADC0->SINGLEDATA),
        numberOfSamplesPerTransfer - 1,
        (void*)secondaryBuffer,
        (void*)&(ADC0->SINGLEDATA),
        numberOfSamplesPerTransfer - 1);

}

bool AudioMoth_enableMicrophone(AM_gainRange_t gainRain, AM_gainSetting_t gain, uint32_t clockDivider, uint32_t acquisitionCycles, uint32_t oversampleRate) {

    /* Check for external microphone */

    bool externalMicrophone = false;

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_2 && hardwareVersion < AM_VERSION_4) {

        GPIO_PinModeSet(JCK_DETECT_GPIOPORT, JCK_DETECT, gpioModeInput, 0);

        GPIO_IntConfig(JCK_DETECT_GPIOPORT, JCK_DETECT, true, true, true);

        externalMicrophone = GPIO_PinInGet(JCK_DETECT_GPIOPORT, JCK_DETECT) == 0;

    }

    if (hardwareVersion >= AM_VERSION_4) {

        GPIO_PinModeSet(JCK_DETECT_ALT_GPIOPORT, JCK_DETECT_ALT, gpioModeInput, 0);

        GPIO_IntConfig(JCK_DETECT_ALT_GPIOPORT, JCK_DETECT_ALT, true, true, true);

        externalMicrophone = GPIO_PinInGet(JCK_DETECT_ALT_GPIOPORT, JCK_DETECT_ALT) == 0;

    }

    /* Enable microphone power */

    if (externalMicrophone) {

        if (hardwareVersion < AM_VERSION_4) {
            GPIO_PinOutClear(JCK_ENABLE_GPIOPORT, JCK_ENABLE_N);
        } else {
            GPIO_PinOutClear(JCK_ENABLE_ALT_GPIOPORT, JCK_ENABLE_ALT_N);
        }

    } else {

        if (hardwareVersion < AM_VERSION_4) GPIO_PinOutClear(VMIC_GPIOPORT, VMIC_ENABLE_N);

    }

    /* Enable VREF power */

    if (hardwareVersion < AM_VERSION_4) GPIO_PinOutSet(VREF_GPIOPORT, VREF_ENABLE);

    /* Set up amplifier stage and the ADC */

    setupOpAmp(gainRain, gain);

    setupADC(clockDivider, acquisitionCycles, oversampleRate);

    return externalMicrophone;

}

void AudioMoth_disableMicrophone(void) {

    /* Check the hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    /* Stop the ADC interrupts */

    ADC_IntDisable(ADC0, ADC_IEN_SINGLE);

    /* Disable interrupts */

    NVIC_ClearPendingIRQ(ADC0_IRQn);

    NVIC_DisableIRQ(ADC0_IRQn);

    /* Stop the DMA transfers */

    DMA_Reset();

    /* Disable internal microphone */

    if (hardwareVersion < AM_VERSION_4) GPIO_PinOutSet(VMIC_GPIOPORT, VMIC_ENABLE_N);

    /* Disable external microphone */

    if (hardwareVersion >= AM_VERSION_2 && hardwareVersion < AM_VERSION_4) {

        GPIO_IntConfig(JCK_DETECT_GPIOPORT, JCK_DETECT, true, true, false);

        GPIO_PinModeSet(JCK_DETECT_GPIOPORT, JCK_DETECT, gpioModeDisabled, 0);

        GPIO_PinOutSet(JCK_ENABLE_GPIOPORT, JCK_ENABLE_N);

    }

    if (hardwareVersion >= AM_VERSION_4) {

        GPIO_IntConfig(JCK_DETECT_ALT_GPIOPORT, JCK_DETECT_ALT, true, true, false);

        GPIO_PinModeSet(JCK_DETECT_ALT_GPIOPORT, JCK_DETECT_ALT, gpioModeDisabled, 0);

        GPIO_PinOutSet(JCK_ENABLE_ALT_GPIOPORT, JCK_ENABLE_ALT_N);

    }

    /* Disable VREF power */

    if (hardwareVersion < AM_VERSION_4) GPIO_PinOutClear(VREF_GPIOPORT, VREF_ENABLE);

    /* Stop the clocks */

    CMU_ClockEnable(cmuClock_DAC0, false);
    CMU_ClockEnable(cmuClock_ADC0, false);
    CMU_ClockEnable(cmuClock_DMA, false);

}

bool AudioMoth_enableExternalSRAM(void) {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return false;

    /* Turn SRAM card on */

    GPIO_PinOutClear(SRAMEN_GPIOPORT, SRAM_ENABLE_N);

    /* Enable the external bus interface */

    enableEBI();

    /* Return success */

    return true;

}

void AudioMoth_disableExternalSRAM(void) {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return;

    /* Turn SRAM card off */

    GPIO_PinOutSet(SRAMEN_GPIOPORT, SRAM_ENABLE_N);

    /* Enable the external bus interface */

    disableEBI();

}

static void enablePrsTimer(uint32_t sampleRate) {

    CMU_ClockEnable(cmuClock_PRS, true);

    CMU_ClockEnable(cmuClock_TIMER2, true);

    /* Connect PRS channel 0 to TIMER overflow */

    PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER2, PRS_CH_CTRL_SIGSEL_TIMER2OF, prsEdgeOff);

    /* Enable TIMER with default settings */

    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

    timerInit.enable = false;

    TIMER_Init(TIMER2, &timerInit);

    /* Configure TIMER to trigger on sampling rate */

    TIMER_TopSet(TIMER2,  CMU_ClockFreqGet(cmuClock_TIMER2) / sampleRate - 1);

    /* Enable Timer on ADC */

    TIMER_Enable(TIMER2, true);

}

static void setupOpAmp(AM_gainRange_t gainRange, AM_gainSetting_t gainSetting) {

    /* Check the hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    /* Start the clock */

    CMU_ClockEnable(cmuClock_DAC0, true);

    /* Define the configuration for OPA1 and OPA2 */

    OPAMP_Init_TypeDef opa1Init = OPA_INIT_INVERTING;

    OPAMP_Init_TypeDef opa2Init = OPA_INIT_INVERTING_OPA2;

    if (hardwareVersion < AM_VERSION_4) {

        opa2Init.outPen = DAC_OPA2MUX_OUTPEN_OUT1;

    } else {

        opa1Init.outMode = opaOutModeAlt;
        opa1Init.outPen = DAC_OPA1MUX_OUTPEN_OUT4;

        opa2Init.outPen = DAC_OPA2MUX_OUTPEN_OUT0;

    }

    /* Set the gain */

    static OPAMP_ResSel_TypeDef opamp1NormalGainRange[] = {opaResSelR2eq4_33R1, opaResSelR2eq7R1, opaResSelR2eq15R1, opaResSelR2eq15R1, opaResSelR2eq15R1};
    static OPAMP_ResSel_TypeDef opamp2NormalGainRange[] = {opaResSelR2eqR1, opaResSelR2eqR1, opaResSelR2eqR1, opaResSelR1eq1_67R1, opaResSelR2eq2R1};

    static OPAMP_ResSel_TypeDef opamp1LowGainRange[] = {opaResSelR2eq0_33R1, opaResSelR2eq0_33R1, opaResSelR2eqR1, opaResSelR2eqR1, opaResSelR2eqR1};
    static OPAMP_ResSel_TypeDef opamp2LowGainRange[] = {opaResSelR2eqR1, opaResSelR1eq1_67R1, opaResSelR2eqR1, opaResSelR1eq1_67R1, opaResSelR2eq2R1};

    OPAMP_ResSel_TypeDef *opamp1Gain = gainRange == AM_LOW_GAIN_RANGE ? opamp1LowGainRange : opamp1NormalGainRange;
    OPAMP_ResSel_TypeDef *opamp2Gain = gainRange == AM_LOW_GAIN_RANGE ? opamp2LowGainRange : opamp2NormalGainRange;

    uint32_t index = MAX(AM_GAIN_LOW, MIN(gainSetting, AM_GAIN_HIGH));

    opa1Init.resSel = opamp1Gain[index];
    opa2Init.resSel = opamp2Gain[index];

    /* Enable OPA1 and OPA2 */

    OPAMP_Enable(DAC0, OPA1, &opa1Init);

    OPAMP_Enable(DAC0, OPA2, &opa2Init);

    /* Disable the clock */

    CMU_ClockEnable(cmuClock_DAC0, false);

}

static void setupADC(uint32_t clockDivider, uint32_t acquisitionCycles, uint32_t oversampleRate) {

    /* Check the hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    /* Start the clock */

    CMU_ClockEnable(cmuClock_ADC0, true);

    /* Configure ADC initialisation structure */

    ADC_Init_TypeDef adcInit = ADC_INIT_DEFAULT;

    if (clockDivider < 1) clockDivider = 1;

    if (clockDivider > 128) clockDivider = 128;

    adcInit.prescale = (clockDivider - 1);

    adcInit.warmUpMode = adcWarmupKeepADCWarm;

    adcInit.timebase = ADC_TimebaseCalc(0);

    adcInit.lpfMode = adcLPFilterRC;

    if (oversampleRate == 128) {
        adcInit.ovsRateSel = adcOvsRateSel128;
    } else if (oversampleRate == 64) {
        adcInit.ovsRateSel = adcOvsRateSel64;
    } else if (oversampleRate == 32) {
        adcInit.ovsRateSel = adcOvsRateSel32;
    } else if (oversampleRate == 16) {
        adcInit.ovsRateSel = adcOvsRateSel16;
    } else if (oversampleRate == 8) {
        adcInit.ovsRateSel = adcOvsRateSel8;
    } else if (oversampleRate == 4) {
        adcInit.ovsRateSel = adcOvsRateSel4;
    } else {
        adcInit.ovsRateSel = adcOvsRateSel2;
    }

    ADC_Init(ADC0, &adcInit);

    /* SCAN mode voltage reference must match the reference selected for SINGLE mode conversions */

    ADC0->SCANCTRL = ADC_SCANCTRL_REF_2V5;

    /* Configure ADC single conversion structure */

    ADC_InitSingle_TypeDef adcSingleInit = ADC_INITSINGLE_DEFAULT;

    adcSingleInit.prsSel = adcPRSSELCh0;
    adcSingleInit.reference = adcRef2V5;

    if (oversampleRate == 1) {

        adcSingleInit.resolution = adcRes12Bit;

    } else {

        adcSingleInit.resolution = adcResOVS;

    }

    if (hardwareVersion < AM_VERSION_4) {

        adcSingleInit.input = adcSingleInpCh0Ch1;

    } else {

        adcSingleInit.input = adcSingleInpCh4Ch5;

    }

    adcSingleInit.prsEnable = true;
    adcSingleInit.diff = true;
    adcSingleInit.rep = false;

    if (acquisitionCycles == 16) {
        adcSingleInit.acqTime = adcAcqTime16;
    } else if (acquisitionCycles == 8) {
        adcSingleInit.acqTime = adcAcqTime8;
    } else if (acquisitionCycles == 4) {
        adcSingleInit.acqTime = adcAcqTime4;
    } else if (acquisitionCycles == 2) {
        adcSingleInit.acqTime = adcAcqTime2;
    } else {
        adcSingleInit.acqTime = adcAcqTime1;
    }

    ADC_InitSingle(ADC0, &adcSingleInit);

}

/* Function to implement a sleeping delay */

void AudioMoth_delay(uint32_t milliseconds) {

    /* Ensure the delay period wont cause the counter to overflow and calculate clock ticks to wait */

    if (milliseconds == 0)  return;

    if (milliseconds > MILLISECONDS_IN_SECOND) milliseconds = MILLISECONDS_IN_SECOND;

    /* Enable clock for TIMER1 */

    CMU_ClockEnable(cmuClock_TIMER1, true);

    /* Initialise TIMER1 */

    TIMER_Init_TypeDef delayInit = TIMER_INIT_DEFAULT;

    delayInit.prescale = timerPrescale1024;

    delayInit.enable = false;

    TIMER_Init(TIMER1, &delayInit);

    /* Set up interrupt and set top */

    TIMER_IntEnable(TIMER1, TIMER_IF_OF);

    NVIC_ClearPendingIRQ(TIMER1_IRQn);

    NVIC_EnableIRQ(TIMER1_IRQn);

    /* Start timer and wait until interrupt occurs */

    delayTimmerRunning = true;

    uint32_t clockTicksToWait = ROUNDED_DIV((CMU_ClockFreqGet(cmuClock_HF) >> timerPrescale1024) * milliseconds, MILLISECONDS_IN_SECOND);

    TIMER_TopSet(TIMER1, clockTicksToWait);

    TIMER_CounterSet(TIMER1, 0);

    TIMER_Enable(TIMER1, true);

    while (delayTimmerRunning) {

        EMU_EnterEM1();

    }

    /* Disable interrupt and reset TIMER1 */

    TIMER_IntDisable(TIMER1, TIMER_IF_OF);

    NVIC_DisableIRQ(TIMER1_IRQn);

    TIMER_Reset(TIMER1);

    /* Disable the clock for TIMER1 */

    CMU_ClockEnable(cmuClock_TIMER1, false);

}

void AudioMoth_sleep(void) {

    EMU_EnterEM1();

}

void AudioMoth_deepSleep(void) {

    EMU_EnterEM2(true);

}

/* Function to power down the device */

void AudioMoth_powerDown() {

    /* Set up GPIO pins */

    setupGPIO();

    /* Disable watch dog timer */

    WDOG_Enable(false);

    /* Disable the BURTC */

    BURTC_Enable(false);

    /* Enter EM4 */

    EMU_EnterEM4();

    while (1) { };

}

void AudioMoth_powerDownAndWakeMilliseconds(uint32_t milliseconds) {

    /* Put GPIO pins in power down state */

    setupGPIO();

    /* Disable watch dog timer */

    WDOG_Enable(false);

    /* CLear BURTC comparison flag */

    BURTC_IntClear(BURTC_IF_COMP0);

    /* Calculate new comparison value */

    uint32_t counterValueToMatch = BURTC_CounterGet();

    uint32_t period = ROUNDED_DIV(milliseconds * AM_BURTC_TICKS_PER_SECOND, MILLISECONDS_IN_SECOND);

    counterValueToMatch += MAX(AM_MINIMUM_POWER_DOWN_TIME, period);

    BURTC_CompareSet(0, counterValueToMatch);

    /* Enable compare interrupt flag */

    BURTC_IntEnable(BURTC_IF_COMP0);

    /* Enter EM4 */

    EMU_EnterEM4();

    while (1) { };

}

void AudioMoth_powerDownAndWake(uint32_t seconds, bool synchronised) {

    if (synchronised == false) {

        AudioMoth_powerDownAndWakeMilliseconds(seconds * MILLISECONDS_IN_SECOND);

    } else {

        /* Put GPIO pins in power down state */

        setupGPIO();

        /* Disable watch dog timer */

        WDOG_Enable(false);

        /* CLear BURTC comparison flag */

        BURTC_IntClear(BURTC_IF_COMP0);

        /* Calculate new comparison value */

        uint32_t counterValueToMatch = BURTC_CounterGet();

        if (seconds == 0) {

            counterValueToMatch += AM_MINIMUM_POWER_DOWN_TIME;

        } else {

            counterValueToMatch += seconds * AM_BURTC_TICKS_PER_SECOND;

            uint32_t offset = counterValueToMatch % AM_BURTC_TICKS_PER_SECOND;

            counterValueToMatch -= offset;

            if (seconds == 1 && offset > AM_BURTC_TICKS_PER_SECOND - AM_MINIMUM_POWER_DOWN_TIME) {

                counterValueToMatch += offset - (AM_BURTC_TICKS_PER_SECOND - AM_MINIMUM_POWER_DOWN_TIME);

            }

        }

        BURTC_CompareSet(0, counterValueToMatch);

        /* Enable compare interrupt flag */

        BURTC_IntEnable(BURTC_IF_COMP0);

        /* Enter EM4 */

        EMU_EnterEM4();

        while (1) { };

    }

}

/* Callback to start the USB reading process when the device is configured */

void stateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState) {

    if (newState == USBD_STATE_CONFIGURED) {

        USBD_Read(HID_EP_OUT, receiveBuffer, AM_USB_BUFFERSIZE, dataReceivedHIDCallback);

        USBD_Read(WEBUSB_EP_OUT, receiveBuffer, AM_USB_BUFFERSIZE, dataReceivedWebUSBCallback);

    } else if (oldState == USBD_STATE_CONFIGURED) {
    
        USBD_AbortTransfer(HID_EP_OUT);

        USBD_AbortTransfer(WEBUSB_EP_OUT);

    }
    
}

/* Callback which provides the WEB USB and USB HID specific descriptors */

int setupCmd(const USB_Setup_TypeDef *setup) {

    int retVal = USB_STATUS_REQ_UNHANDLED;

    if (setup->Type == USB_SETUP_TYPE_STANDARD) {

        if (setup->bRequest == GET_DESCRIPTOR) {
        
            switch (setup->wValue >> 8) {

                case USB_HID_REPORT_DESCRIPTOR:

                    USBD_Write( 0, (void*)HID_ReportDescriptor, SL_MIN(sizeof(HID_ReportDescriptor), setup->wLength), NULL );

                    retVal = USB_STATUS_OK;

                    break;

                case USB_HID_DESCRIPTOR:

                    USBD_Write( 0, (void*)HID_Descriptor, SL_MIN(sizeof(HID_Descriptor), setup->wLength), NULL );

                    retVal = USB_STATUS_OK;

                    break;

                case USB_BOS_DESCRIPTOR:

                    USBD_Write(0, (void*)BOS_Descriptor, SL_MIN(sizeof(BOS_Descriptor), setup->wLength), NULL);

                    retVal = USB_STATUS_OK;

                    break;

            }
        
        }
    
    }

    if (setup->bmRequestType == WEB_USB_REQUEST_TYPE) {

        if (setup->bRequest == WEB_USB_URL_REQUEST && setup->wIndex == WEB_USB_URL_INDEX) {

            USBD_Write(0, (void*)URL_Descriptor, SL_MIN(sizeof(URL_Descriptor), setup->wLength), NULL);

            retVal = USB_STATUS_OK;

        }

        if (setup->bRequest == WEB_USB_MSFT_REQUEST && setup->wIndex == WEB_USB_MSFT_INDEX) {

            USBD_Write(0, (void*)MICROSOFT_Descriptor, SL_MIN(sizeof(MICROSOFT_Descriptor), setup->wLength), NULL);

            retVal = USB_STATUS_OK;

        }

    }

    return retVal;

}

/* CRC update function */

static inline uint16_t updateCRC(uint16_t crc, uint32_t incr) {

    uint16_t xor = crc >> 15;

    uint16_t out = crc << 1;

    if (incr) out++;

    if (xor) out ^= FIRMWARE_CRC_POLY;

    return out;

}

/* Function held in SRAM to clear user flash page */

SL_RAMFUNC_DEFINITION_BEGIN
static void __attribute__ ((noinline)) clearUserDataPageInFlash() {

    /* Unlock the internal flash for erasing and writing */

    MSC->LOCK = MSC_UNLOCK_CODE;

    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    /* Erase the internal flash page */

    MSC->ADDRB = AM_FLASH_USER_DATA_ADDRESS;

    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

    MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

    while (MSC->STATUS & MSC_STATUS_BUSY);

    /* Write the internal flash page */

    for (uint32_t j = 0; j < AM_FIRMWARE_PAGE_SIZE; j += 4) {

        MSC->WDATA = 0x00;

        MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

        while (MSC->STATUS & MSC_STATUS_BUSY);

    }

    /* Lock the internal flash */

    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

    MSC->LOCK = 0;

}
SL_RAMFUNC_DEFINITION_END

/* Function held in SRAM to write firmware to internal flash */

SL_RAMFUNC_DEFINITION_BEGIN
static void __attribute__ ((noinline)) writeFirmwareToInternalFlash() {

    /* Unlock the internal flash for erasing and writing */

    MSC->LOCK = MSC_UNLOCK_CODE;

    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    /* Iterate through firmware pages */

    for (uint32_t i = 0; i < AM_FIRMWARE_TOTAL_SIZE; i += AM_FIRMWARE_PAGE_SIZE) {

        /* Erase the internal flash page */

        MSC->ADDRB = AM_FIRMWARE_START_ADDRESS + i;

        MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

        MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

        while (MSC->STATUS & MSC_STATUS_BUSY);

        /* Write the internal flash page */

        for (uint32_t j = 0; j < AM_FIRMWARE_PAGE_SIZE; j += 4) {

            MSC->WDATA = *(uint32_t*)(AM_EXTERNAL_SRAM_START_ADDRESS + i + j);

            MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

            while (MSC->STATUS & MSC_STATUS_BUSY);

        }

    }

    /* Lock the internal flash */

    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

    MSC->LOCK = 0;

    /* Reset to start the new firmware */

    NVIC_SystemReset();

}
SL_RAMFUNC_DEFINITION_END

/* Callback on receipt of message from the USB host */

void handleUSBPacket() {

    uint8_t receivedMessageType = receiveBuffer[0];

    /* Write default returned message */

    memset(transmitBuffer, 0, AM_USB_BUFFERSIZE);

    transmitBuffer[0] = receivedMessageType;

    /* Respond to message type */

    switch(receivedMessageType) {

        case AM_USB_MSG_TYPE_GET_TIME: {

            /* Requests the current time from the device */

            uint32_t time;

            AudioMoth_getTime(&time, NULL);

            *(uint32_t*)(transmitBuffer + 1) = time;

            } break;

        case AM_USB_MSG_TYPE_SET_TIME: {

            /* Provides the time to set the device RTC */

            uint32_t time = *(uint32_t*)(receiveBuffer + 1);

            *(uint32_t*)(transmitBuffer + 1) = time;

            AudioMoth_setTime(time, 0);

            AudioMoth_blinkDuringUSB(USB_SET_TIME_BLINK);

            } break;

        case AM_USB_MSG_TYPE_GET_UID:

            /* Requests the UID of the device */

            memcpy(transmitBuffer + 1, (void*)AM_UNIQUE_ID_START_ADDRESS, AM_UNIQUE_ID_SIZE_IN_BYTES);

            break;

        case AM_USB_MSG_TYPE_GET_BATTERY: {

            /* Requests the state of the battery */

            uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

            AM_batteryState_t batteryState = AudioMoth_getBatteryState(supplyVoltage);

            *(AM_batteryState_t*)(transmitBuffer + 1) = batteryState;

            } break;

        case AM_USB_MSG_TYPE_GET_APP_PACKET:

            /* Requests application specific packet from the device */

            AudioMoth_usbApplicationPacketRequested(AM_USB_MSG_TYPE_GET_APP_PACKET, transmitBuffer, AM_USB_BUFFERSIZE);

            break;

        case AM_USB_MSG_TYPE_SET_APP_PACKET:

            /* Provides the application specific packet */

            AudioMoth_usbApplicationPacketReceived(AM_USB_MSG_TYPE_SET_APP_PACKET, receiveBuffer, transmitBuffer, AM_USB_BUFFERSIZE);

            break;

        case AM_USB_MSG_TYPE_GET_FIRMWARE_VERSION: {

            /* Provides the application firmware version */

            uint8_t *firmwareNumber = NULL;

            AudioMoth_usbFirmwareVersionRequested(&firmwareNumber);

            if (firmwareNumber) memcpy(transmitBuffer + 1, firmwareNumber, AM_FIRMWARE_VERSION_LENGTH);

            } break;

        case AM_USB_MSG_TYPE_GET_FIRMWARE_DESCRIPTION: {

            /* Provides the application firmware description */

            uint8_t *firmwareDescription = NULL;

            AudioMoth_usbFirmwareDescriptionRequested(&firmwareDescription);

            if (firmwareDescription) memcpy(transmitBuffer + 1, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

            } break;

        case AM_USB_MSG_TYPE_QUERY_SERIAL_BOOTLOADER:

            /* Query support for automatic bootloader entry */

            transmitBuffer[1] = true;

            break;

        case AM_USB_MSG_TYPE_ENTER_SERIAL_BOOTLOADER:

            /* Enter bootloader after sending response */

            enterSerialBootloader = true;

            transmitBuffer[1] = true;

            break;

        case AM_USB_MSG_TYPE_QUERY_USBHID_BOOTLOADER: {

            /* Query support for USB HID bootloader */

            AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

            transmitBuffer[1] = hardwareVersion < AM_VERSION_4;

            } break;

        case AM_USB_MSG_TYPE_ENTER_USBHID_BOOTLOADER: {

            /* Check support for USB HID bootloader */

            AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

            if (hardwareVersion >= AM_VERSION_4) break;

            /* Handle USB HID bootloader message */

            uint8_t bootloaderMessageType = receiveBuffer[1];

            transmitBuffer[1] = bootloaderMessageType;

            /* Respond to message type */

            switch(bootloaderMessageType) {

                case AM_BOOTLOADER_GET_VERSION:

                    transmitBuffer[2] = 0x01;

                    break;

                case AM_BOOTLOADER_INITIALISE_SRAM: {

                    /* Enable the external SRAM */

                    bool success = AudioMoth_enableExternalSRAM();

                    /* Erase SRAM buffer */

                    if (success) {

                        uint8_t *buffer = (uint8_t*)AM_EXTERNAL_SRAM_START_ADDRESS;

                        memset(buffer, 0xFF, AM_FIRMWARE_TOTAL_SIZE);

                        transmitBuffer[2] = true;

                    }

                    } break;

                case AM_BOOTLOADER_CLEAR_USER_PAGE: {

                    /* Clear SRAM buffer */

                    clearUserDataPageInFlash();

                    /* Check the page contents */

                    bool success = true;

                    for (uint32_t i = 0; i < AM_FLASH_USER_SIZE_IN_BYTES; i += 1) {

                        uint8_t byte = *(uint8_t*)(AM_FLASH_USER_DATA_ADDRESS + i);

                        success &= byte == 0x00;

                    }

                    transmitBuffer[2] = success;

                    } break;

                case AM_BOOTLOADER_SET_SRAM_FIRMWARE_PACKET: {

                    /* Copy firmware bytes to SRAM buffer */

                    uint8_t *buffer = (uint8_t*)AM_EXTERNAL_SRAM_START_ADDRESS;

                    usbMessageSetFirmwarePacket_t *msg = (usbMessageSetFirmwarePacket_t*)(receiveBuffer + 2);

                    if (msg->offset + msg->length < AM_FIRMWARE_TOTAL_SIZE) memcpy(buffer + msg->offset, receiveBuffer + sizeof(usbMessageSetFirmwarePacket_t) + 2, msg->length);

                    memcpy(transmitBuffer + 2, receiveBuffer + 2, msg->length + sizeof(usbMessageSetFirmwarePacket_t));

                    } break;

                case AM_BOOTLOADER_CALC_SRAM_FIRMWARE_CRC:
                case AM_BOOTLOADER_CALC_FLASH_FIRMWARE_CRC:

                    if (shouldCalculateCRC) break;

                    firmwareStartAddress = bootloaderMessageType == AM_BOOTLOADER_CALC_SRAM_FIRMWARE_CRC ? (uint8_t*)AM_EXTERNAL_SRAM_START_ADDRESS : (uint8_t*)AM_FIRMWARE_START_ADDRESS;

                    completedCalculateCRC = false;

                    shouldCalculateCRC = true;

                    break;

                case AM_BOOTLOADER_GET_FIRMWARE_CRC: {

                    /* Return the CRC value of the firmware */

                    transmitBuffer[2] = completedCalculateCRC;

                    if (completedCalculateCRC) *(uint16_t*)(transmitBuffer + 3) = currentCRC;

                    } break;

                case AM_BOOTLOADER_FLASH_FIRMWARE:

                    /* Flash firmware after sending response */

                    shouldFlashFirmware = true;

                    transmitBuffer[2] = true;

                    break;

                default:

                    break;

            }

        } break;
        
        default:

            break;

    }

}

/* Callback to handle data received from USB HID request */

int dataReceivedHIDCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    /* Handle response */

    handleUSBPacket();

    /* Send the response */

    USBD_Write(HID_EP_IN, transmitBuffer, AM_USB_BUFFERSIZE, dataSentHIDCallback);

    return USB_STATUS_OK;

}

/* Callback to handle data received from WEB USB request */

int dataReceivedWebUSBCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    /* Handle response */

    handleUSBPacket();

    /* Send the response */

    USBD_Write(WEBUSB_EP_IN, transmitBuffer, AM_USB_BUFFERSIZE, dataSentWebUSBCallback);

    return USB_STATUS_OK;

}

/* Callback on completion of USB HID data send. Used to request next read */

int dataSentHIDCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    USBD_Read(HID_EP_OUT, receiveBuffer, AM_USB_BUFFERSIZE, dataReceivedHIDCallback);

    return USB_STATUS_OK;

}

/* Callback on completion of WEB USB data send. Used to request next read */

int dataSentWebUSBCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    USBD_Read(WEBUSB_EP_OUT, receiveBuffer, AM_USB_BUFFERSIZE, dataReceivedWebUSBCallback);

    return USB_STATUS_OK;

}

/* Function to handle USB from the application */

void AudioMoth_blinkDuringUSB(uint32_t milliseconds) {

    usbLoopCounter = milliseconds;

}

void AudioMoth_handleUSB(void) {

    /* Configure data input pin */

    GPIO_PinModeSet(USB_DATA_GPIOPORT, USB_P, gpioModeInputPull, 0);

    /* Enable RTC for watch dog and BURTC overflow */

    AudioMoth_startRealTimeClock(AM_USB_EM2_RTC_WAKEUP_INTERVAL);

    /* Enable the USB interface */

    USBD_Init(&initstruct);

    /* Stay within this busy loop while the switch is in USB */

    usbLoopCounter = 0;

    while (AudioMoth_getSwitchPosition() == AM_SWITCH_USB && !enterSerialBootloader && !shouldFlashFirmware) {

        /* Turn green LED on to indicate activity */

        if (GPIO_PinInGet(USB_DATA_GPIOPORT, USB_P)) {
            
            if (usbLoopCounter == 0) AudioMoth_setGreenLED(true);

            AudioMoth_delay(1);

        }

        if (usbLoopCounter > 0) usbLoopCounter -= 1;

        /* Calcualte CRC */

        if (shouldCalculateCRC) {

            currentCRC = 0;
            
            for (uint32_t i = 0; i < AM_FIRMWARE_TOTAL_SIZE; i += 1) {

                uint32_t byte = firmwareStartAddress[i];

                for (uint32_t j = 0x80; j > 0; j >>= 1) currentCRC = updateCRC(currentCRC, byte & j);

            }

            for (uint32_t j = 0; j < 16; j += 1) currentCRC = updateCRC(currentCRC, 0);

            completedCalculateCRC = true;

            shouldCalculateCRC = false;

        }

        /* Handle BURTC overflow */

        AudioMoth_checkAndHandleTimeOverflow();

        /* Turn off green LED */

        AudioMoth_setGreenLED(false);

        /* Enter low power standby if USB is unplugged */

        if (USBD_SafeToEnterEM2()) {

            EMU_EnterEM2(true);

        }

    }

    /* Wait for last response to be sent */

    AudioMoth_delay(100);

    /* Disable USB */

    USBD_AbortAllTransfers();

    USBD_Stop();

    USBD_Disconnect();

    /* Disable RTC */

    AudioMoth_stopRealTimeClock();

    /* Disable the data input pin */

    GPIO_PinModeSet(USB_DATA_GPIOPORT, USB_P, gpioModeDisabled, 0);   

    /* Jump directly to the serial bootloader */

    if (enterSerialBootloader) {

        /* Disable watch dog timer */

        WDOG_Enable(false);

        /* Pull bootloader pin high */

        GPIO->ROUTE &= ~GPIO_ROUTE_SWCLKPEN;

        GPIO_PinModeSet(gpioPortF, 0, gpioModePushPull, 1);

        /* Jump to bootloader */

        __asm (

            /* Define the bootloader and vector table addresses */

            ".equ BOOTLOADER_ADDRESS, 0x00000000\n\t"

            ".equ SCB_VTOR, (0xE000E000 + 0x0D00 + 0x008)\n\t"

            /* Load the bootloader address */

            "ldr r0, =BOOTLOADER_ADDRESS\n\t"

            /* Set the vector table */

            "ldr r1, =SCB_VTOR\n\t"
            "str r0, [r1]\n\t"

            /* Set the stack pointer */

            "ldr r1, [r0]\n\t"
            "msr msp, r1\n\t"
            "msr psp, r1\n\t"

            /* Jump into the bootloader */

            "ldr r1, [r0, #4]\n\t"
            "mov pc, r1\n\t"

        );

    }

    /* Flash the firmware from the external SRAM */

    if (shouldFlashFirmware) {

        AudioMoth_setBothLED(true);

        writeFirmwareToInternalFlash();

    }

}

/* Function to handle hardware version sensing */

static AM_hardwareVersion_t senseHardwareVersion() {

   /* Enable ADC clock */

   CMU_ClockEnable(cmuClock_ADC0, true);

   /* Initialise ADC */

   ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

   ADC_Init(ADC0, &init);

   /* Initialise ADC for single measurement */

   ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

   singleInit.input = adcSingleInpCh5;
   singleInit.reference = adcRefVDD;
   singleInit.acqTime = adcAcqTime32;

   ADC_InitSingle(ADC0, &singleInit);

   /* Perform conversion */

   ADC_Start(ADC0, adcStartSingle);

   while (ADC0->STATUS & ADC_STATUS_SINGLEACT) { };

   uint32_t value = ADC_DataSingleGet(ADC0);

   /* Reset ADC */

   ADC_Reset(ADC0);

   /* Disable ADC clock */

   CMU_ClockEnable(cmuClock_ADC0, false);

   /* Return the hardware version */

   AM_hardwareVersion_t hardwareVersion = value < 256 ? AM_VERSION_1 : value < 512 ? AM_VERSION_2 : value < 768 ? AM_VERSION_3 : AM_VERSION_4;

   return hardwareVersion;

}

/* Functions to handle supply voltage monitoring */

void AudioMoth_enableSupplyMonitor() {

    /* Set up VCMP */

    VCMP_Init_TypeDef vcmpInit = VCMP_INIT_DEFAULT;

    vcmpInit.halfBias = false;
    vcmpInit.lowPowerRef = false;
    vcmpInit.warmup = vcmpWarmTime256Cycles;

    /* Enable VCMP clock */

    CMU_ClockEnable(cmuClock_VCMP, true);

    /* Initialise the VCMP */

    VCMP_Init(&vcmpInit);

}

void AudioMoth_disableSupplyMonitor() {

    /* Disable VCMP */

    VCMP_Disable();

    /* Disable voltage comparator clock*/

    CMU_ClockEnable(cmuClock_VCMP, false);

}

static inline void updateSupplyMonitorThresholdLevel(uint32_t level) {

    /* Disable VCMP */

    VCMP_Disable();

    /* Set new threshold value */

    level = MAX(MINIMIMUM_COMPARATOR_LEVEL, MIN(MAXIMIMUM_COMPARATOR_LEVEL, level));

    VCMP->INPUTSEL = (VCMP->INPUTSEL & ~(_VCMP_INPUTSEL_TRIGLEVEL_MASK)) | (level << _VCMP_INPUTSEL_TRIGLEVEL_SHIFT);

    /* Enable and wait until ready */

    VCMP_Enable();

    while (!(VCMP->STATUS & VCMP_STATUS_VCMPACT)) { };

}

void AudioMoth_setSupplyMonitorThreshold(uint32_t supplyVoltage) {

    uint32_t level = ROUNDED_DIV(supplyVoltage - VCMP_VOLTAGE_OFFSET, VCMP_VOLTAGE_INCREMENT);

    updateSupplyMonitorThresholdLevel(level);

}

bool AudioMoth_isSupplyAboveThreshold() {

    return (VCMP->STATUS & VCMP_STATUS_VCMPOUT);

}

/* Functions to handle the battery monitor */

void AudioMoth_enableBatteryMonitor() {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return;

    /* Enable battery monitor pin */

    GPIO_PinOutSet(BAT_MON_GPIOPORT, BAT_MON_ENABLE);

    /* Initialise the ACMP */

    ACMP_Init_TypeDef acmpInit = ACMP_INIT_DEFAULT;

    acmpInit.warmTime = acmpWarmTime256;
    acmpInit.hysteresisLevel = acmpHysteresisLevel0;

    /* Enable ACMP clock */

    CMU_ClockEnable(cmuClock_ACMP0, true);

    /* Initialise the ACMP */

    ACMP_Init(ACMP0, &acmpInit);

    /* Set the ACMP channel */

    ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel0);

}

static inline void updateBatteryMonitorThresholdLevel(uint32_t level) {

    /* Disable ACMP */

    ACMP_Disable(ACMP0);

    /* Set new threshold value */

    level = MAX(MINIMIMUM_COMPARATOR_LEVEL, MIN(MAXIMIMUM_COMPARATOR_LEVEL, level));

    ACMP0->INPUTSEL = (ACMP0->INPUTSEL & ~(_ACMP_INPUTSEL_VDDLEVEL_MASK)) | (level << _ACMP_INPUTSEL_VDDLEVEL_SHIFT);

    /* Enable and wait until ready */

    ACMP_Enable(ACMP0);

    while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) { };

}

void AudioMoth_setBatteryMonitorThreshold(uint32_t batteryVoltage, uint32_t supplyVoltage) {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return;

    /* Set battery monitor threshold */

    uint32_t level = ROUNDED_DIV(MAXIMIMUM_COMPARATOR_LEVEL * batteryVoltage / BATTERY_MONITOR_DIVIDER, supplyVoltage);

    updateBatteryMonitorThresholdLevel(level);

}

bool AudioMoth_isBatteryAboveThreshold() {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return false;

    /* Return battery monitor state */

    return (ACMP0->STATUS & ACMP_STATUS_ACMPOUT);

}

void AudioMoth_disableBatteryMonitor() {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return;

    /* Disable ACMP */

    ACMP_Disable(ACMP0);

    /* Disable ACMP clock */

    CMU_ClockEnable(cmuClock_ACMP0, false);

    /* Disable battery monitor pin */

    GPIO_PinOutClear(BAT_MON_GPIOPORT, BAT_MON_ENABLE);

}

/* Functions to handle supply voltage and battery voltage / state reporting */

uint32_t AudioMoth_getSupplyVoltage() {

    /* Enable supply monitor */

    AudioMoth_enableSupplyMonitor();

    /* Find level at which supply voltage exceeds threshold */

    uint32_t level = MAXIMIMUM_SUPPLY_MONITOR_LEVEL;

    while (level > MINIMIMUM_SUPPLY_MONITOR_LEVEL) {

        updateSupplyMonitorThresholdLevel(level);

        if (AudioMoth_isSupplyAboveThreshold()) break;

        level -= 1;
    }

    /* Disable supply monitor */

    AudioMoth_disableSupplyMonitor();

    /* Calculate voltage midway between levels */

    uint32_t supplyVoltage = VCMP_VOLTAGE_OFFSET + level * VCMP_VOLTAGE_INCREMENT + VCMP_VOLTAGE_INCREMENT / 2;

    return supplyVoltage;

}

AM_extendedBatteryState_t AudioMoth_getExtendedBatteryState(uint32_t supplyVoltage) {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return AM_EXT_BAT_LOW;

    /* Enable battery monitor */

    AudioMoth_enableBatteryMonitor();

    /* Find level at which battery voltage exceeds threshold */

    uint32_t batteryVoltage = MAXIMIMUM_BATTERY_MONITOR_VOLTAGE;

    AM_extendedBatteryState_t extendedBatteryState = AM_EXT_BAT_FULL;

    while (extendedBatteryState > AM_EXT_BAT_LOW) {

        AudioMoth_setBatteryMonitorThreshold(batteryVoltage, supplyVoltage);

        if (AudioMoth_isBatteryAboveThreshold()) break;

        extendedBatteryState -= 1;

        batteryVoltage -= AM_BATTERY_STATE_INCREMENT;

    }

    /* Disable battery monitor */

    AudioMoth_disableBatteryMonitor();

    /* Return battery state */

    return extendedBatteryState;

}

AM_batteryState_t AudioMoth_getBatteryState(uint32_t supplyVoltage) {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return AM_BATTERY_LOW;

    /* Get the battery state */

    AM_extendedBatteryState_t extendedBatteryState = AudioMoth_getExtendedBatteryState(supplyVoltage);

    AM_batteryState_t batteryState = extendedBatteryState < AM_EXT_BAT_3V6 ? AM_BATTERY_LOW : extendedBatteryState - (AM_BATTERY_STATE_OFFSET - AM_EXT_BAT_STATE_OFFSET) / AM_BATTERY_STATE_INCREMENT;

    return batteryState;

}

/* Functions to handle temperature monitoring */

void AudioMoth_enableTemperature() {

    /* Enable ADC clock */

    CMU_ClockEnable(cmuClock_ADC0, true);

    /* Initialise ADC */

    ADC_Init_TypeDef adcInit = ADC_INIT_DEFAULT;

    adcInit.ovsRateSel = adcOvsRateSel16;
    adcInit.timebase = ADC_TimebaseCalc(0);
    adcInit.prescale = ADC_PrescaleCalc(400000, 0);

    ADC_Init(ADC0, &adcInit);

    /* Initialise ADC for single measurement */

    ADC_InitSingle_TypeDef adcSingleInit = ADC_INITSINGLE_DEFAULT;

    adcSingleInit.reference = adcRef1V25;
    adcSingleInit.acqTime = adcAcqTime32;
    adcSingleInit.input = adcSingleInpTemp;

    ADC_InitSingle(ADC0, &adcSingleInit);

}

void AudioMoth_disableTemperature() {

    /* Reset ADC */

    ADC_Reset(ADC0);

    /* Disable ADC clock */

    CMU_ClockEnable(cmuClock_ADC0, false);

}

int32_t AudioMoth_getTemperature() {

    /* Start ADC measurement and wait for completion */

    ADC_Start(ADC0, adcStartSingle);

    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) { };

    /* Calculate temperature */

    int32_t adcSample = ADC_DataSingleGet(ADC0);

    uint32_t CAL_TEMP_0 = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);

    if ((CAL_TEMP_0 == 0xFF) || (CAL_TEMP_0 == 0xFFF)) return -100000;

    int32_t ADC0_TEMP_0_READ_1V25 = ((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

    int32_t temperature = MILLIDEGREES_IN_DEGREE * CAL_TEMP_0;

    temperature += ROUNDED_DIV(MILLIDEGREES_IN_DEGREE * GRADIENT_MULTIPLIER * (ADC0_TEMP_0_READ_1V25 - adcSample), TEMPERATURE_GRADIENT);

    return temperature;

}

/* Function to query the switch position */

AM_switchPosition_t AudioMoth_getSwitchPosition(void) {

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (GPIO_PinInGet(SWITCH_1_GPIOPORT, SWITCH_1_SENSE) == 0) return AM_SWITCH_DEFAULT;

    if (hardwareVersion < AM_VERSION_3) {

        if (GPIO_PinInGet(SWITCH_2_GPIOPORT, SWITCH_2_SENSE) == 1)  return AM_SWITCH_USB;

        return AM_SWITCH_CUSTOM;

    } else {

        if (GPIO_PinInGet(SWITCH_2_GPIOPORT, SWITCH_2_SENSE) == 1)  return AM_SWITCH_CUSTOM;

        return AM_SWITCH_USB;

    }

}

/* Functions to handle storing and retrieving values in the application backup registers */

void AudioMoth_storeInBackupDomain(uint32_t number, uint32_t data) {

    if (number < AM_BURTC_TOTAL_REGISTERS - AM_BURTC_RESERVED_REGISTERS) {

        BURTC_RetRegSet(AM_BURTC_RESERVED_REGISTERS + number, data);

    }

}

uint32_t AudioMoth_retreiveFromBackupDomain(uint32_t number) {

    if (number < AM_BURTC_TOTAL_REGISTERS - AM_BURTC_RESERVED_REGISTERS) {

        return BURTC_RetRegGet(AM_BURTC_RESERVED_REGISTERS + number);

    } else {

        return 0;

    }

}

/* Functions to handle access to the flash user data page */

SL_RAMFUNC_DEFINITION_BEGIN
bool AudioMoth_writeToFlashUserDataPage(uint8_t *data, uint32_t length) {

    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_ATOMIC();

    MSC->LOCK = MSC_UNLOCK_CODE;

    MSC_Init();

    MSC_Status_TypeDef status = MSC_ErasePage((uint32_t*)AM_FLASH_USER_DATA_ADDRESS);

    if (status == mscReturnOk) {

        status = MSC_WriteWord((uint32_t*)AM_FLASH_USER_DATA_ADDRESS, data, length);

    }

    MSC->LOCK = 0;

    CORE_EXIT_ATOMIC();

    return status == mscReturnOk;

}
SL_RAMFUNC_DEFINITION_END

/* Functions to handle setting and querying of time */

bool AudioMoth_hasTimeBeenSet(void) {

    return BURTC_RetRegGet(AM_BURTC_CLOCK_SET_FLAG) == AM_BURTC_CANARY_VALUE;

}

static void setTime(uint32_t time, uint32_t milliseconds) {

    uint32_t ticks = ROUNDED_DIV(AM_BURTC_TICKS_PER_SECOND * milliseconds, MILLISECONDS_IN_SECOND);

    uint64_t intendedCounter = AM_BURTC_TICKS_PER_SECOND * (uint64_t)time + ticks;

    uint64_t offset = intendedCounter - (uint64_t)BURTC_CounterGet();

    BURTC_RetRegSet(AM_BURTC_TIME_OFFSET_HIGH, (uint32_t)(offset >> 32));

    BURTC_RetRegSet(AM_BURTC_TIME_OFFSET_LOW, (uint32_t)(offset & 0xFFFFFFFF));

    BURTC_RetRegSet(AM_BURTC_CLOCK_SET_FLAG, AM_BURTC_CANARY_VALUE);

}

static void getTime(uint32_t *time, uint32_t *milliseconds) {

    uint64_t offset =  (uint64_t)BURTC_RetRegGet(AM_BURTC_TIME_OFFSET_HIGH) << 32;

    offset += (uint64_t)BURTC_RetRegGet(AM_BURTC_TIME_OFFSET_LOW);

    uint64_t currentCounter = offset + BURTC_CounterGet();

    if (time != NULL) {

        *time = currentCounter / AM_BURTC_TICKS_PER_SECOND;

    }

    if (milliseconds != NULL) {

        uint32_t ticks = currentCounter % AM_BURTC_TICKS_PER_SECOND;

        *milliseconds = ROUNDED_DIV(MILLISECONDS_IN_SECOND * ticks, AM_BURTC_TICKS_PER_SECOND);

    }

}

static void handleTimeOverflow(void) {

    uint32_t offsetHigh = BURTC_RetRegGet(AM_BURTC_TIME_OFFSET_HIGH);

    BURTC_RetRegSet(AM_BURTC_TIME_OFFSET_HIGH, offsetHigh + 1);

}

void AudioMoth_setTime(uint32_t time, uint32_t milliseconds) {

    setTime(time, milliseconds);

    if (BURTC_IntGet() & BURTC_IF_OF) {

        handleTimeOverflow();

        setTime(time, milliseconds);

        BURTC_IntClear(BURTC_IF_OF);

    }

}

void AudioMoth_getTime(uint32_t *time, uint32_t *milliseconds) {

    getTime(time, milliseconds);

    if (BURTC_IntGet() & BURTC_IF_OF) {

        handleTimeOverflow();

        getTime(time, milliseconds);

        BURTC_IntClear(BURTC_IF_OF);

    }

}

/* Functions to initialise, feed and query the watch dog timer */

static void setupWatchdogTimer(void) {

    WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;

    wdogInit.em2Run = true;
    wdogInit.em3Run = true;
    wdogInit.perSel = wdogPeriod_64k;

    WDOG_Init(&wdogInit);

}

bool AudioMoth_hasWatchdogResetOccurred(void) {

    return BURTC_RetRegGet(AM_BURTC_WATCH_DOG_FLAG) == AM_BURTC_CANARY_VALUE;

}

void AudioMoth_feedWatchdog(void) {

    WDOG_Feed();

}

void AudioMoth_startWatchdog(void) {

    WDOG_Enable(true);

}

void AudioMoth_stopWatchdog(void) {

    WDOG_Enable(false);

}

/* Real time clock */

static void startRealTimeClock(uint32_t ticks) {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    /* Enable LF oscillator and RTC clock */

    if (hardwareVersion < AM_VERSION_4) CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);

    CMU_ClockEnable(cmuClock_RTC, true);

    /* Configure RTC */

    RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

    RTC_CompareSet(0, ticks);

    RTC_IntEnable(RTC_IEN_COMP0);

    NVIC_ClearPendingIRQ(RTC_IRQn);

    NVIC_EnableIRQ(RTC_IRQn);

    RTC_Init(&rtcInit);

}

void AudioMoth_startRealTimeClock(uint32_t seconds) {

    if (seconds == 0) return;

    if (seconds > SECONDS_IN_MINUTE) seconds = SECONDS_IN_MINUTE;

    uint32_t ticks = seconds * AM_LFXO_LFRCO_TICKS_PER_SECOND;

    startRealTimeClock(ticks);

}

void AudioMoth_startRealTimeClockMilliseconds(uint32_t milliseconds) {

    if (milliseconds == 0) return;

    if (milliseconds > MILLISECONDS_IN_SECOND) milliseconds = MILLISECONDS_IN_SECOND;

    uint32_t ticks = ROUNDED_DIV(milliseconds * AM_LFXO_LFRCO_TICKS_PER_SECOND, MILLISECONDS_IN_SECOND);

    startRealTimeClock(ticks);

}

void AudioMoth_stopRealTimeClock(void) {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    /* Stop RTC interrupts */

    RTC_IntDisable(RTC_IEN_COMP0);

    NVIC_DisableIRQ(RTC_IRQn);

    RTC_Reset();

    /* Disable LF oscillator and RTC clock */

    CMU_ClockEnable(cmuClock_RTC, false);

    if (hardwareVersion < AM_VERSION_4) CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);

}

/* Function to check and handle timer overflow */

void AudioMoth_checkAndHandleTimeOverflow(void) {

    if (BURTC_IntGet() & BURTC_IF_OF) {

        handleTimeOverflow();

        BURTC_IntClear(BURTC_IF_OF);

    }

}

/* Time function for FAT file system */

DWORD get_fattime(void) {

    int8_t timezoneHours = 0;

    int8_t timezoneMinutes = 0;

    AudioMoth_timezoneRequested(&timezoneHours, &timezoneMinutes);

    uint32_t currentTime;

    AudioMoth_getTime(&currentTime, NULL);

    time_t fatTime = currentTime + timezoneHours * 60 * 60 + timezoneMinutes * 60;

    struct tm timePtr;
    gmtime_r(&fatTime, &timePtr);

    return (((unsigned int)timePtr.tm_year - 208) << 25) |
            (((unsigned int)timePtr.tm_mon + 1 ) << 21) |
            ((unsigned int)timePtr.tm_mday << 16) |
            ((unsigned int)timePtr.tm_hour << 11) |
            ((unsigned int)timePtr.tm_min << 5) |
            ((unsigned int)timePtr.tm_sec >> 1);

}

/* Functions to handle changing state of the red and green LED */

void AudioMoth_setRedLED(bool state) {

    GPIO_PinModeSet(LED_GPIOPORT, RED_LED, gpioModePushPull, state);

}

void AudioMoth_setBothLED(bool state) {

    GPIO_PinModeSet(LED_GPIOPORT, RED_LED, gpioModePushPull, state);
    GPIO_PinModeSet(LED_GPIOPORT, GREEN_LED, gpioModePushPull, state);

}

void AudioMoth_setGreenLED(bool state) {

    GPIO_PinModeSet(LED_GPIOPORT, GREEN_LED, gpioModePushPull, state);

}

/* Functions to handle file system */

bool AudioMoth_enableFileSystem(AM_sdCardSpeed_t speed) {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return false;

    /* Turn SD card on */

    GPIO_PinOutClear(SDEN_GPIOPORT, SD_ENABLE_N);

    /* Initialise MicroSD driver */

    if (speed == AM_SD_CARD_HIGH_SPEED) MICROSD_DoubleSpiClkFast();

    MICROSD_Init();

    /* Check SD card status */

    DSTATUS resCard = disk_initialize(0);

    if (resCard == STA_NOINIT || resCard == STA_NODISK || resCard == STA_PROTECT) {
        return false;
    }

    /* Initialise file system */

    if (f_mount(&fatfs, "", 1) != FR_OK) {
        return false;
    }

    /* Return success */

    return true;

}

void AudioMoth_disableFileSystem(void) {

    /* Check hardware version */

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

    if (hardwareVersion >= AM_VERSION_4) return;

    /* Disable the SD card pins */

    MICROSD_Deinit();

    /* Turn SD card off*/

    GPIO_PinOutSet(SDEN_GPIOPORT, SD_ENABLE_N);

}

bool AudioMoth_doesFileExist(char *filename){

    FRESULT res = f_stat(filename, NULL);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_openFile(char *filename) {

    /* Open a file for writing. Overwrite existing file with the same name */

    FRESULT res = f_open(&file, filename,  FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_appendFile(char *filename) {

    /* Open the file for writing. Append existing file with the same name */

    FRESULT res = f_open(&file, filename,  FA_OPEN_ALWAYS | FA_WRITE | FA_READ);

    if (res != FR_OK) {
        return false;
    }

    res = f_lseek(&file, f_size(&file));

    if (res != FR_OK) {
        f_close(&file);
        return false;
    }

    return true;

}

bool AudioMoth_openFileToRead(char *filename) {

    FRESULT res = f_open(&file, filename,  FA_READ);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_readFile(char *buffer, uint32_t bufferSize) {

    FRESULT res = f_read(&file, buffer, bufferSize, &bw);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_seekInFile(uint32_t position) {

    FRESULT res = f_lseek(&file, position);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_writeToFile(void *bytes, uint16_t bytesToWrite) {

    FRESULT res = f_write(&file, bytes, bytesToWrite, &bw);

    if ((res != FR_OK) || (bytesToWrite != bw)) {
        return false;
    }

    return true;

}

bool AudioMoth_renameFile(char *originalFilename, char *newFilename) {

    FRESULT res = f_rename(originalFilename, newFilename);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_syncFile(void) {

    FRESULT res = f_sync(&file);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_closeFile(void) {

    FRESULT res = f_close(&file);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_doesDirectoryExist(char *folderName){

    FRESULT res = f_stat(folderName, NULL);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_makeDirectory(char *folderName) {

    FRESULT res = f_mkdir(folderName);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

/* Functions to enable and disable EBI */

static void enableEBI(void) {

    /* Enable clocks */

    CMU_ClockEnable(cmuClock_EBI, true);

    /* Enable EBI AD0..07 data pins*/

    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD00, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD01, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD02, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD03, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD04, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD05, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD06, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD07, gpioModePushPull, 0);

    /* Enable EBI AD08..15 address pins*/

    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD08, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD09, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD10, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD11, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD12, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD13, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD14, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD15, gpioModePushPull, 0);

    /* Enable EBI A16..24 extension address pins*/

    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A08, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A09, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A10, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A11, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A12, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A13, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A14, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A15, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A16, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A17, gpioModePushPull, 0);

    /* Enable EBI CS0-CS1 */

    GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL1, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL2, gpioModePushPull, 0);

    /* Enable EBI WEN/OEN */

    GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_OE, gpioModePushPull, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_WE, gpioModePushPull, 0);

    /* Configure EBI controller, changing default values */

    EBI_Init_TypeDef ebiInit = EBI_INIT_DEFAULT;

    ebiInit.mode = ebiModeD8A8;
    ebiInit.banks = EBI_BANK0;
    ebiInit.csLines = EBI_CS0 | EBI_CS1;
    ebiInit.readHalfRE = true;

    ebiInit.aLow = ebiALowA8;
    ebiInit.aHigh = ebiAHighA18;

    /* Address Setup and hold time */

    ebiInit.addrHoldCycles  = 0;
    ebiInit.addrSetupCycles = 0;

    /* Read cycle times */

    ebiInit.readStrobeCycles = 3;
    ebiInit.readHoldCycles   = 1;
    ebiInit.readSetupCycles  = 2;

    /* Write cycle times */

    ebiInit.writeStrobeCycles = 6;
    ebiInit.writeHoldCycles   = 0;
    ebiInit.writeSetupCycles  = 0;

    ebiInit.location = ebiLocation1;

    /* Configure EBI */

    EBI_Init(&ebiInit);

}

static void disableEBI(void) {

    /* Disable EBI AD0..07 data pins*/

    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD00, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD01, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD02, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD03, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD04, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD05, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD06, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD07, gpioModeDisabled, 0);

    /* Disable EBI AD08..15 address pins*/

    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD08, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD09, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD10, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD11, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD12, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD13, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD14, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD15, gpioModeDisabled, 0);

    /* Disable EBI A16..24 extension address pins*/

    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A16, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A17, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A18, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A19, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A21, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A22, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A23, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A24, gpioModeDisabled, 0);

    /* Disable EBI CS0-CS1 */

    GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL1, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL2, gpioModeDisabled, 0);

    /* Disable EBI WEN/OEN */

    GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_OE, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_WE, gpioModeDisabled, 0);

    /* Turn off EBI clock */

    CMU_ClockEnable(cmuClock_EBI, false);

}

/* Function configure GPIO pins */

static void setupGPIO(void) {

    AM_hardwareVersion_t hardwareVersion = BURTC_RetRegGet(AM_BURTC_HARDWARE_VERSION);

	/* GPIO A */

	GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD09, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD10, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD11, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD12, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD13, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD14, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD15, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortA, 7, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortA, 8, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortA, 9, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortA, 10, gpioModeDisabled, 0);

    if (hardwareVersion >= AM_VERSION_4) {
	    GPIO_PinModeSet(VREF_GPIOPORT, VREF_ENABLE, gpioModeDisabled, 0);
    } else {
        GPIO_PinModeSet(VREF_GPIOPORT, VREF_ENABLE, gpioModePushPull, 0);
    }
	
    GPIO_PinModeSet(gpioPortA, 12, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortA, 13, gpioModeDisabled, 0);
	GPIO_PinModeSet(JCK_DETECT_GPIOPORT, JCK_DETECT, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD08, gpioModeDisabled, 0);

	/* GPIO B */

	GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A16, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A17, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 2, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 3, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 4, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 5, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 6, gpioModeDisabled, 0);

    if (hardwareVersion >= AM_VERSION_4) {
        GPIO_PinModeSet(gpioPortB, 7, gpioModeDisabled, 0);
        GPIO_PinModeSet(gpioPortB, 8, gpioModeDisabled, 0);
    }

	GPIO_PinModeSet(gpioPortB, 9, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 10, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 11, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 12, gpioModeDisabled, 0);

	/* GPIO C */

	GPIO_PinModeSet(gpioPortC, 0, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortC, 1, gpioModeDisabled, 0);

    if (hardwareVersion >= AM_VERSION_4) {
	    GPIO_PinModeSet(BAT_MON_GPIOPORT, BAT_MON_ENABLE, gpioModeDisabled, 0);
    } else {
        GPIO_PinModeSet(BAT_MON_GPIOPORT, BAT_MON_ENABLE, gpioModePushPull, 0);
    }

    if (hardwareVersion < AM_VERSION_4) {
        GPIO_PinModeSet(JCK_ENABLE_ALT_GPIOPORT, JCK_ENABLE_ALT_N, gpioModeDisabled, 0);
    } else {
        GPIO_PinModeSet(JCK_ENABLE_ALT_GPIOPORT, JCK_ENABLE_ALT_N,  gpioModePushPull, 1);
    }

	GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A15, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A09, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A10, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortC, 11, gpioModeDisabled, 0);

	/* GPIO D */

    GPIO_PinModeSet(JCK_DETECT_ALT_GPIOPORT, JCK_DETECT_ALT, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortD, 1, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortD, 2, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortD, 3, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortD, 4, gpioModeDisabled, 0);
	GPIO_PinModeSet(VERSION_CONTROL_GPIOPORT, VERSION_CONTROL, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL1, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL2, gpioModeDisabled, 0);

    if (hardwareVersion >= AM_VERSION_4) {
	    GPIO_PinModeSet(SRAMEN_GPIOPORT, SRAM_ENABLE_N, gpioModeDisabled, 0);
    } else {
        GPIO_PinModeSet(SRAMEN_GPIOPORT, SRAM_ENABLE_N, gpioModePushPull, 1);
    }

    if (hardwareVersion >= AM_VERSION_4) {
        GPIO_PinModeSet(SDEN_GPIOPORT, SD_ENABLE_N, gpioModeDisabled, 0);
    } else {
	    GPIO_PinModeSet(SDEN_GPIOPORT, SD_ENABLE_N, gpioModePushPull, 1);
    }

	/* GPIO E */

     if (hardwareVersion >= AM_VERSION_4) {
	   GPIO_PinModeSet(VMIC_GPIOPORT, VMIC_ENABLE_N, gpioModeDisabled, 0);
    } else {
        GPIO_PinModeSet(VMIC_GPIOPORT, VMIC_ENABLE_N, gpioModePushPull, 1);
    }

	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A08, gpioModeDisabled, 0);

    if (hardwareVersion < AM_VERSION_2 || hardwareVersion >= AM_VERSION_4) {
        GPIO_PinModeSet(JCK_ENABLE_GPIOPORT, JCK_ENABLE_N, gpioModeDisabled, 0);
    } else {
        GPIO_PinModeSet(JCK_ENABLE_GPIOPORT, JCK_ENABLE_N, gpioModePushPull, 1);
    }

	GPIO_PinModeSet(gpioPortE, 3, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A11, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A12, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A13, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A14, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD00, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD01, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD02, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD03, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD04, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD05, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD06, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD07, gpioModeDisabled, 0);

	/* GPIO F */

	GPIO_PinModeSet(gpioPortF, 3, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortF, 4, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortF, 5, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortF, 6, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortF, 7, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_WE, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_OE, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortF, 12, gpioModeDisabled, 0);

    /* Enable GPIO state retention in EM4 */

	GPIO->CTRL = GPIO_CTRL_EM4RET;

}

/* Enable SWO output for debugging */

int _write(int file, const char *ptr, int len) {

    int x;

    for (x = 0; x < len; x++) ITM_SendChar (*ptr++);

    return (len);

}

void AudioMoth_setupSWOForPrint(void) {

    /* Enable GPIO clock. */

    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

    /* Enable Serial wire output pin */

    GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;

    /* Set location 0 */

    GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

    /* Enable output on pin - GPIO Port F, Pin 2 */

    GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);

    GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;

    /* Enable debug clock AUXHFRCO */

    CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

    /* Wait until clock is ready */

    while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

    /* Enable trace in core debug */

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    ITM->LAR  = 0xC5ACCE55;

    ITM->TER  = 0x0;

    ITM->TCR  = 0x0;

    TPI->SPPR = 2;

    TPI->ACPR = 0xf;

    ITM->TPR  = 0x0;

    DWT->CTRL = 0x400003FE;

    ITM->TCR  = 0x0001000D;

    TPI->FFCR = 0x00000100;

    ITM->TER  = 0x1;

}

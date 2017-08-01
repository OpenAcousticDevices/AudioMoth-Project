/****************************************************************************
 * audioMoth.c
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "em_acmp.h"
#include "em_adc.h"
#include "em_burtc.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_dma.h"
#include "em_ebi.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_opamp.h"
#include "em_rmu.h"
#include "em_timer.h"
#include "em_usart.h"
#include "em_usb.h"
#include "em_wdog.h"

#include "ff.h"
#include "diskio.h"
#include "dmactrl.h"
#include "microsd.h"

#include "pinouts.h"
#include "usbconfig.h"
#include "usbcallbacks.h"
#include "usbdescriptors.h"

#include "audioMoth.h"

/*  Define RTC backup register constants */

#define AM_BURTC_COUNTER                          0
#define AM_BURTC_CLOCK_SET_FLAG                   1
#define AM_BURTC_WATCH_DOG_FLAG                   2
#define AM_BURTC_INITIAL_POWER_UP_FLAG            3

#define AM_BURTC_CANARY_VALUE                     0x11223344

#define AM_BURTC_TOTAL_REGISTERS                  128
#define AM_BURTC_RESERVED_REGISTERS               8

/*  Define oscillator constants */

#define AM_LFXO_TICKS_PER_SECOND                  256
#define AM_MINIMUM_POWER_DOWN_TIME                4

/* Define USB message types and declare buffers */

#define AM_USB_BUFFERSIZE                         64

#define AM_USB_MSG_TYPE_GET_TIME                  0x01
#define AM_USB_MSG_TYPE_SET_TIME                  0x02
#define AM_USB_MSG_TYPE_GET_UID                   0x03
#define AM_USB_MSG_TYPE_GET_BATTERY               0x04
#define AM_USB_MSG_TYPE_GET_APP_PACKET            0x05
#define AM_USB_MSG_TYPE_SET_APP_PACKET            0x06

STATIC_UBUF(receiveBuffer, AM_USB_BUFFERSIZE);
STATIC_UBUF(transmitBuffer, AM_USB_BUFFERSIZE);

/* SD card variables */

static FATFS fatfs;
static FIL file;
static UINT bw;

/* DMA variables */

static DMA_CB_TypeDef cb;
static uint16_t numberOfSamplesPerTransfer;

/* Function prototypes */

static void setupGPIO(void);
static void enableEBI(void);
static void disableEBI(void);
static void setupBackupRTC(void);
static void setupBackupDomain(void);
static void setupWatchdogTimer(void);
static void setupOpAmp(uint32_t gain);
static bool batteryIsAboveVoltageThreshold(uint32_t vddLevelDivider);
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

    /* Enable clock to low energy modules */

    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Store the cause of the last reset, and clear the reset cause register */

    unsigned long resetCause = RMU_ResetCauseGet();

    RMU_ResetCauseClear();

    /* If this is a start from power-off initialise low frequency oscillator and set up BURTC */

    if ((resetCause & RMU_RSTCAUSE_EM4WURST) == 0) {

        /* Start LFXO and wait until it is stable */

        CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

        /* Setup up back up domain EM4 */

        setupBackupDomain();

        /* Setup up backup RTC */

        setupBackupRTC();

        /* Reset the time set flag and the counter */

        BURTC_RetRegSet(AM_BURTC_CLOCK_SET_FLAG, 0);

        BURTC_RetRegSet(AM_BURTC_COUNTER, 0);

        /* Set the initial power up flag */

        BURTC_RetRegSet(AM_BURTC_INITIAL_POWER_UP_FLAG,  AM_BURTC_CANARY_VALUE);

    } else {

        /* Clear the initial power up flag */

        BURTC_RetRegSet(AM_BURTC_INITIAL_POWER_UP_FLAG,  0);

    }

    /* Record that a watch dog timer reset has occurred */

    if (resetCause & RMU_RSTCAUSE_WDOGRST) {

        BURTC_RetRegSet(AM_BURTC_WATCH_DOG_FLAG, AM_BURTC_CANARY_VALUE);

    }

    /* Put GPIO pins in correct state */

    setupGPIO();

    /* Enable interrupt on USB switch position to wake from EM2 */

    GPIO_PinModeSet(USB_GPIOPORT, USB_PIN, gpioModeInput, 0);

    GPIO_PinModeSet(SWITCH_GPIOPORT, SWITCH_PIN, gpioModeInput, 0);

    GPIO_IntConfig(USB_GPIOPORT, USB_PIN, true, true, true);

    GPIO_IntConfig(SWITCH_GPIOPORT, SWITCH_PIN, true, true, true);

    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);

    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

    /* Start the watch dog time */

    setupWatchdogTimer();

    WDOG_Enable(true);

}

bool AudioMoth_isInitialPowerUp(void) {

    return BURTC_RetRegGet(AM_BURTC_INITIAL_POWER_UP_FLAG) == AM_BURTC_CANARY_VALUE;

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

    if (frequency < AM_HFXO) {

        CMU_HFRCOBandSet(frequency);

    } else {

        CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);

    }

}

void AudioMoth_selectHFRCO() {

    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

}

void AudioMoth_disableHFRCO(void) {

    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

}

void AudioMoth_calibrateHFRCO(uint32_t frequency) {

    /* Set up initial limits for binary search */

    uint32_t lower = 0x00;
    uint32_t upper = 0xFF;

    uint32_t mid = CMU_OscillatorTuningGet(cmuOsc_HFRCO);

    /* Calculate expected number of up cycles */

    uint32_t downCycles = 0xC000;

    uint32_t targetUpCycles = (float)downCycles * (float)frequency / (float)CMU_ClockFreqGet(cmuClock_HF);

    /* Main binary search */

    while (lower < upper) {

        uint32_t actualUpCycles = CMU_Calibrate(downCycles, cmuOsc_HFRCO);

        if (actualUpCycles > targetUpCycles) {

            /* HFRCO is running too fast, search next in lower half */

            upper = mid - 1;

        } else {

            /* HFRCO is running too slow, search next in upper half */

            lower = mid + 1;

        }

        /* Calculate and set next mid point */

        mid = (upper + lower) / 2;

        CMU_OscillatorTuningSet(cmuOsc_HFRCO, mid);



    }

}

uint32_t AudioMoth_getClockFrequency(AM_clockFrequency_t frequency) {

    switch (frequency) {
        case AM_HFRCO_1MHZ:
            return 1200000;
        case AM_HFRCO_7MHZ:
            return 6600000;
        case AM_HFRCO_11MHZ:
            return 11000000;
        case AM_HFRCO_14MHZ:
            return 14000000;
        case AM_HFRCO_21MHZ:
            return 21000000;
        case AM_HFRCO_28MHZ:
            return 28000000;
        case AM_HFXO:
            return 48000000;
    }

    return 48000000;

}

/* Interrupt handler for switch change events, microphone samples, timer overflow and DMA transfers */

void GPIO_EVEN_IRQHandler(void) {

    /* Clear the GPIO interrupt flag */

    GPIO_IntClear(GPIO_IntGet());

    /* Call the interrupt handler */

    AudioMoth_handleSwitchInterrupt();

}

void ADC0_IRQHandler(void) {

    /* Clear the ADC0 interrupt flag */

    ADC_IntClear(ADC0, ADC_IF_SINGLE);

    /* Send the sample to the interrupt handler */

    int16_t sample = ADC_DataSingleGet(ADC0);

    AudioMoth_handleMicrophoneInterrupt(sample);

    /* Feed the watch dog timer */

    WDOG_Feed();

}

void TIMER1_IRQHandler(void) {

  /* Clear the TIMER1 overflow flag */

  TIMER_IntClear(TIMER1, TIMER_IF_OF);

}

static void transferComplete(unsigned int channel, bool primary, void *user) {

    int16_t *nextBuffer = NULL;

    AudioMoth_handleDirectMemoryAccessInterrupt(primary, &nextBuffer);

    /* Re-activate the DMA */

    DMA_RefreshPingPong(channel,
        primary,
        false,
        (void*)nextBuffer,
        NULL,
        numberOfSamplesPerTransfer - 1,
        false);

    /* Feed the watch dog timer */

    WDOG_Feed();

}

/* Set up backup domain */

static void setupBackupDomain(void) {

    /* Initialise GPIO, BURTC and EM4 registers */

    EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;

    em4Init.vreg = true;
    em4Init.lockConfig = true;
    em4Init.buRtcWakeup = true;
    em4Init.osc = emuEM4Osc_LFXO;

    /* Unlock configuration */

    EMU_EM4Lock(false);

    EMU_EM4Init(&em4Init);

    /* Enable access to BURTC registers */

    RMU_ResetControl(rmuResetBU, false);

    /* Lock configuration */

    EMU_EM4Lock(true);

}

/* Configure RTC */

static void setupBackupRTC(void) {

    /* Set up BURTC to count and wake from EM4 */

    BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;

    burtcInit.mode = burtcModeEM4;
    burtcInit.clkSel = burtcClkSelLFXO;
    burtcInit.clkDiv = burtcClkDiv_128;
    burtcInit.timeStamp = false;
    burtcInit.compare0Top = false;
    burtcInit.enable = false;
    burtcInit.lowPowerMode = burtcLPDisable;

    BURTC_Init(&burtcInit);

    /* Enable the timer */

    BURTC_Enable(true);

}

/* Functions to initialise the microphone */

void AudioMoth_startMicrophoneSamples(void) {

    /* Start the ADC samples */

    ADC_Start(ADC0, adcStartSingle);

}

void AudioMoth_initialiseMicrophoneInterupts(void) {

    /* Enable ADC interrupt vector in NVIC */

    ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

    NVIC_ClearPendingIRQ(ADC0_IRQn);

    NVIC_EnableIRQ(ADC0_IRQn);

}

void AudioMoth_initialiseDirectMemoryAccess(int16_t *primaryBuffer, int16_t *secondaryBuffer, uint16_t numberOfSamples) {

    numberOfSamplesPerTransfer = numberOfSamples;

    if (numberOfSamplesPerTransfer > 1024) {

        numberOfSamplesPerTransfer = 1024;

    }

    /* Start the clock */

    CMU_ClockEnable(cmuClock_DMA, true);

    /* Initialise the DMA structure */

    DMA_Init_TypeDef dmaInit;

    dmaInit.hprot        = 0;
    dmaInit.controlBlock = dmaControlBlock;

    DMA_Init(&dmaInit);

    /* Setting up call-back function */

    cb.cbFunc  = transferComplete;
    cb.userPtr = NULL;

    /* Setting up channel */

    DMA_CfgChannel_TypeDef chnlCfg;

    chnlCfg.highPri   = false;
    chnlCfg.enableInt = true;
    chnlCfg.select    = DMAREQ_ADC0_SINGLE;
    chnlCfg.cb        = &cb;

    DMA_CfgChannel(0, &chnlCfg);

    /* Setting up channel descriptor */

    DMA_CfgDescr_TypeDef descrCfg;

    descrCfg.dstInc  = dmaDataInc2;
    descrCfg.srcInc  = dmaDataIncNone;
    descrCfg.size    = dmaDataSize2;
    descrCfg.arbRate = dmaArbitrate1;
    descrCfg.hprot   = 0;

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

void AudioMoth_enableMicrophone(uint32_t gain, uint32_t clockDivider, uint32_t acquisitionCycles, uint32_t oversampleRate) {

    /* Enable microphone and VREF power */

    GPIO_PinOutClear(VMIC_GPIOPORT, VMIC_EN_N);

    GPIO_PinOutSet(VREF_GPIOPORT, VREF_EN);

    /* Set up amplifier stage and the ADC */

    setupOpAmp(gain);

    setupADC(clockDivider, acquisitionCycles, oversampleRate);

}

void AudioMoth_disableMicrophone(void) {

    /* Stop the ADC interrupts */

    ADC_IntDisable(ADC0, ADC_IEN_SINGLE);

    /* Stop the DMA transfers */

    DMA_Reset();

    /* Disable microphone and VREF power */

    GPIO_PinOutSet(VMIC_GPIOPORT, VMIC_EN_N);

    GPIO_PinOutClear(VREF_GPIOPORT, VREF_EN);

    /* Stop the clocks */

    CMU_ClockEnable(cmuClock_DAC0, false);
    CMU_ClockEnable(cmuClock_ADC0, false);
    CMU_ClockEnable(cmuClock_DMA, false);

}

void AudioMoth_enableExternalSRAM(void) {

    /* Turn SRAM card on */

    GPIO_PinOutClear(SRAMEN_GPIOPORT, SRAM_ENABLE_N);

    /* Enable the external bus interface */

    enableEBI();

}

void AudioMoth_disableExternalSRAM(void) {

    /* Turn SRAM card off */

    GPIO_PinOutSet(SRAMEN_GPIOPORT, SRAM_ENABLE_N);

    /* Enable the external bus interface */

    disableEBI();

}

uint32_t AudioMoth_calculateSampleRate(uint32_t frequency, uint32_t clockDivider, uint32_t acquisitionCycles, uint32_t oversampleRate) {

    if (acquisitionCycles != 16 && acquisitionCycles != 8 && acquisitionCycles != 4 && acquisitionCycles != 2) {
        acquisitionCycles = 1;
    }

    if (oversampleRate != 128 && oversampleRate != 64 && oversampleRate != 32 && oversampleRate != 16 && oversampleRate != 8 && oversampleRate != 4) {
        oversampleRate = 2;
    }

    if (clockDivider > 128) {
        clockDivider = 128;
    }

    if (clockDivider < 1) {
        clockDivider = 1;
    }

    uint32_t numerator = frequency / clockDivider;

    uint32_t denominator = 2 + (acquisitionCycles + 12) * oversampleRate;

    return numerator / denominator;

}


static void setupOpAmp(uint32_t gain) {

    /* Start the clock */

    CMU_ClockEnable(cmuClock_DAC0, true);

    /* Define the configuration for OPA1 and OPA2 */

    OPAMP_Init_TypeDef configuration1 = OPA_INIT_INVERTING;

    OPAMP_Init_TypeDef configuration2 = OPA_INIT_INVERTING_OPA2;

    configuration2.outPen = DAC_OPA2MUX_OUTPEN_OUT1;

    if (gain == 4) {

        configuration1.resSel = opaResSelR2eq15R1;
        configuration2.resSel = opaResSelR2eq2R1;

    } else if (gain == 3) {

        configuration1.resSel = opaResSelR2eq15R1;
        configuration2.resSel = opaResSelR1eq1_67R1;

    } else if (gain == 2) {

        configuration1.resSel = opaResSelR2eq15R1;
        configuration2.resSel = opaResSelR2eqR1;

    } else if (gain == 1) {

        configuration1.resSel = opaResSelR2eq7R1;
        configuration2.resSel = opaResSelR2eqR1;

    } else {

        configuration1.resSel = opaResSelR2eq4_33R1;
        configuration2.resSel = opaResSelR2eqR1;

    }

    /* Enable OPA1 and OPA2 */

    OPAMP_Enable(DAC0, OPA1, &configuration1);

    OPAMP_Enable(DAC0, OPA2, &configuration2);

}

static void setupADC(uint32_t clockDivider, uint32_t acquisitionCycles, uint32_t oversampleRate) {

    /* Start the clock */

    CMU_ClockEnable(cmuClock_ADC0, true);

    /* Configure ADC initialisation structure */

    ADC_Init_TypeDef adcInit = ADC_INIT_DEFAULT;

    adcInit.timebase = ADC_TimebaseCalc(0);

    if (clockDivider > 128) {
        clockDivider = 128;
    }

    if (clockDivider < 1) {
        clockDivider = 1;
    }

    adcInit.prescale = (clockDivider - 1);

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

    /* Configure ADC single conversion structure */

    ADC_InitSingle_TypeDef adcInitSingle = ADC_INITSINGLE_DEFAULT;

    adcInitSingle.reference = adcRefVDD;
    adcInitSingle.resolution = adcResOVS;
    adcInitSingle.input = adcSingleInpCh0Ch1;
    adcInitSingle.diff = true;
    adcInitSingle.rep = true;

    if (acquisitionCycles == 16) {
        adcInitSingle.acqTime = adcAcqTime16;
    } else if (acquisitionCycles == 8) {
        adcInitSingle.acqTime = adcAcqTime8;
    } else if (acquisitionCycles == 4) {
        adcInitSingle.acqTime = adcAcqTime4;
    } else if (acquisitionCycles == 2) {
        adcInitSingle.acqTime = adcAcqTime2;
    } else {
        adcInitSingle.acqTime = adcAcqTime1;
    }

    ADC_InitSingle(ADC0, &adcInitSingle);

}

/* Function to implement a busy delay */

void AudioMoth_delay(uint16_t milliseconds) {

    /* Ensure the delay period wont cause the counter to overflow and calculate clock ticks to wait */

    if (milliseconds == 0) {

        return;

    }

    if (milliseconds > 1000) {

        milliseconds = 1000;

    }

    /* Enable clock for TIMER1 */

    CMU_ClockEnable(cmuClock_TIMER1, true);

    /* Initialise and start TIMER1 */

    TIMER_Init_TypeDef delayInit = TIMER_INIT_DEFAULT;

    delayInit.enable = false;
    delayInit.prescale = timerPrescale1024;

    TIMER_Init(TIMER1, &delayInit);

    /* Set up interrupt and set top */

    TIMER_IntEnable(TIMER1, TIMER_IF_OF);

    NVIC_ClearPendingIRQ(TIMER1_IRQn);

    NVIC_EnableIRQ(TIMER1_IRQn);

    /* Start timer and wait until counter value is over the threshold */

    uint32_t clockTicksToWait = (CMU_ClockFreqGet(cmuClock_HF) >> timerPrescale1024) * milliseconds / 1000;

    TIMER_TopSet(TIMER1, clockTicksToWait);

    TIMER_CounterSet(TIMER1, 1);

    TIMER_Enable(TIMER1, true);

    while (TIMER_CounterGet(TIMER1) > 0) {

        EMU_EnterEM1();

    }

    /* Disable interrupt and reset TIMER1 */

    NVIC_DisableIRQ(TIMER1_IRQn);

    TIMER_Reset(TIMER1);

    /* Disable the clock for TIMER1 */

    CMU_ClockEnable(cmuClock_TIMER1, false);

}

void AudioMoth_sleep(void) {

    EMU_EnterEM1();

}

/* Function to power down the device */

void AudioMoth_powerDown() {

    /* Set up GPIO pins */

    setupGPIO();

    /* Disable watchdog timer */

    WDOG_Enable(false);

    /* Disable the BURTC */

    BURTC_Enable(false);

    /* Enter EM4 */

    EMU_EnterEM4();

    while (1) {};

}

void AudioMoth_powerDownAndWake(uint32_t seconds, bool synchronised) {

    /* Put GPIO pins in power down state */

    setupGPIO();

    /* Disable watch dog timer */

    WDOG_Enable(false);

    /* CLear BURTC comparison flag */

    BURTC_IntClear(BURTC_IF_COMP0);

    /* Calculate new comparison value */

    uint32_t currentCounterValue = BURTC_CounterGet();

    if (seconds == 0) {

        BURTC_CompareSet(AM_BURTC_COUNTER, currentCounterValue + AM_MINIMUM_POWER_DOWN_TIME);

    } else {

        uint32_t counterValueToMatch = currentCounterValue + seconds * AM_LFXO_TICKS_PER_SECOND;

        if (synchronised) {

            counterValueToMatch -= counterValueToMatch & (AM_LFXO_TICKS_PER_SECOND - 1);

        }

        BURTC_CompareSet(AM_BURTC_COUNTER, counterValueToMatch + AM_MINIMUM_POWER_DOWN_TIME);

    }

    /* Enable compare interrupt flag */

    BURTC_IntEnable(BURTC_IF_COMP0);

    /* Enter EM4 */

    EMU_EnterEM4();

    while (1) {};

}

/* Callback which provides the HID specific descriptors */

int setupCmd( const USB_Setup_TypeDef *setup ) {

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
            }
        
        }
    
    }

    return retVal;

}

/* Callback to start the USB reading process when the device is configured */

void stateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState) {
    if (newState == USBD_STATE_CONFIGURED) {
        USBD_Read(EP_OUT, receiveBuffer, AM_USB_BUFFERSIZE, dataReceivedCallback);
    }
}

/* Callback on completion of data send. Used to request next read */

int dataSentCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {
    USBD_Read(EP_OUT, receiveBuffer, AM_USB_BUFFERSIZE, dataReceivedCallback);
    return USB_STATUS_OK;
}

/* Callback on receipt of message from the USB host */

int dataReceivedCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    uint8_t receivedMessageType = receiveBuffer[0];

    /* Write default returned message */

    memset(transmitBuffer, 0, AM_USB_BUFFERSIZE);

    transmitBuffer[0] = receivedMessageType;

    /* Respond to message type */

    uint32_t timeNow;

    AM_batteryState_t batteryState;

    switch(receivedMessageType) {

        case AM_USB_MSG_TYPE_GET_TIME:

            /* Requests the current time from the device */

            timeNow = AudioMoth_getTime();

            memcpy(transmitBuffer + 1, &timeNow, 4);

            break;

        case AM_USB_MSG_TYPE_SET_TIME:

            /* Provides the time to set the device RTC */

            memcpy(&timeNow, receiveBuffer + 1, 4);

            memcpy(transmitBuffer + 1, &timeNow, 4);

            AudioMoth_setTime(timeNow);

            break;

        case AM_USB_MSG_TYPE_GET_UID:

            /* Requests the UID of the device */

            memcpy(transmitBuffer + 1, (void*)AM_UNIQUE_ID_START_ADDRESS, AM_UNIQUE_ID_SIZE_IN_BYTES);

            break;

        case AM_USB_MSG_TYPE_GET_BATTERY:

            /* Requests the state of the battery */

            batteryState = AudioMoth_getBatteryState();

            memcpy(transmitBuffer + 1, &batteryState, 1);

            break;

        case AM_USB_MSG_TYPE_GET_APP_PACKET:

            /* Requests application specific packet from the device */

            AudioMoth_usbApplicationPacketRequested(AM_USB_MSG_TYPE_GET_APP_PACKET, transmitBuffer, AM_USB_BUFFERSIZE);

            break;

        case AM_USB_MSG_TYPE_SET_APP_PACKET:

            /* Provides the application specific packet */

            AudioMoth_usbApplicationPacketReceived(AM_USB_MSG_TYPE_SET_APP_PACKET, receiveBuffer, transmitBuffer, AM_USB_BUFFERSIZE);

            break;

        default:

            break;

    }

    /* Send the response */

    USBD_Write(EP_IN, transmitBuffer, AM_USB_BUFFERSIZE, dataSentCallback);

    return USB_STATUS_OK;

}

void AudioMoth_handleUSB(void) {

    /* Configure data input pin */

    GPIO_PinModeSet(USB_DATA_GPIOPORT, USB_P, gpioModeInput, 0);

    /* Disable the watch dog timer */

    WDOG_Enable(false);

    /* Enable the USB interface */

    USBD_Init(&initstruct);

    /* Stay within this busy loop while the switch is in USB */

    while (AudioMoth_getSwitchPosition() == AM_SWITCH_USB) {

        if (GPIO_PinInGet(USB_DATA_GPIOPORT, USB_P)) {

            AudioMoth_setGreenLED(true);

            AudioMoth_delay(1);

            AudioMoth_setGreenLED(false);

            AudioMoth_delay(1);

        }

        if (USBD_SafeToEnterEM2()) {

            EMU_EnterEM2(true);

        }

    }

    /* Disable USB */

    USBD_AbortAllTransfers();
    USBD_Stop();
    USBD_Disconnect();

    /* Re-enable the watch dog timer */

    WDOG_Enable(true);

    /* Disable the data input pin */

    GPIO_PinModeSet(USB_DATA_GPIOPORT, USB_P, gpioModeDisabled, 0);

}

/* Function to get the battery power */

AM_batteryState_t AudioMoth_getBatteryState() {

    AM_batteryState_t batteryState = AM_BATTERY_LOW;

	/* Initialise max and min voltage levels */

	uint32_t minBatteryThreshold = 34;                  // 3V3 * 2 * 34 / 63 = 3.6V

    uint32_t maxBatteryThreshold = 48;                  // 3V3 * 2 * 48 / 63 = 5.0V

	/* Turn battery monitor on */

    GPIO_PinOutSet(BAT_MON_GPIOPORT, BAT_MON_EN);

	/* Enable comparator clock */

	CMU_ClockEnable(cmuClock_ACMP0, true);

	for (uint32_t threshold = minBatteryThreshold; threshold <= maxBatteryThreshold; threshold += 1) {

		if (batteryIsAboveVoltageThreshold(threshold)) {

		    batteryState += 1;

		} else {

		    break;

		}

	}

	/* Turn Battery Monitor off*/

    GPIO_PinOutClear(BAT_MON_GPIOPORT, BAT_MON_EN);

	/* Disable comparator clock*/

	CMU_ClockEnable(cmuClock_ACMP0, false);

	return batteryState;

}

static bool batteryIsAboveVoltageThreshold(uint32_t threshold) {

    bool aboveVoltageThreshold = false;

    /* Initialise and initialise the ACMP */

    ACMP_Init_TypeDef acmp_init = ACMP_INIT_DEFAULT;

    acmp_init.vddLevel = threshold;

    acmp_init.hysteresisLevel = acmpHysteresisLevel2;

    ACMP_Init(ACMP0, &acmp_init);

    /* Set the ACMP channel */

    ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel0);

    /* Wait for warm up */

    while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT));

    if (ACMP0->STATUS & ACMP_STATUS_ACMPOUT) {

        aboveVoltageThreshold = true;

    }

    ACMP_Disable(ACMP0);

    return aboveVoltageThreshold;

}

/* Function to query the switch position */

AM_switchPosition_t AudioMoth_getSwitchPosition(void) {

    /* Test conditions */

    if (GPIO_PinInGet(USB_GPIOPORT, USB_PIN) == 1) {

        return AM_SWITCH_USB;

    }

    if (GPIO_PinInGet(SWITCH_GPIOPORT, SWITCH_PIN) == 0) {

        return AM_SWITCH_DEFAULT;

    }

    return AM_SWITCH_CUSTOM;

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

/* Functions to handle setting and querying of time */

void AudioMoth_setTime(uint32_t time) {

    BURTC_RetRegSet(AM_BURTC_COUNTER, time);

    BURTC_RetRegSet(AM_BURTC_CLOCK_SET_FLAG, AM_BURTC_CANARY_VALUE);

    BURTC_RetRegSet(AM_BURTC_WATCH_DOG_FLAG, 0);

    BURTC_CounterReset();

}

bool AudioMoth_hasTimeBeenSet(void) {

    return BURTC_RetRegGet(AM_BURTC_CLOCK_SET_FLAG) == AM_BURTC_CANARY_VALUE;

}

uint32_t AudioMoth_getTime(void) {

    return BURTC_RetRegGet(AM_BURTC_COUNTER) + BURTC_CounterGet() / AM_LFXO_TICKS_PER_SECOND;

}

/* Functions to initialise, feed and query the watch dog timer */

static void setupWatchdogTimer(void) {

    WDOG_Init_TypeDef init = WDOG_INIT_DEFAULT;

    init.em2Run = true;
    init.em3Run = true;
    init.perSel = wdogPeriod_64k;

    WDOG_Init(&init);

}

bool AudioMoth_hasWatchdogResetOccured(void) {

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

/* Time function for FAT file system */

DWORD get_fattime(void) {

    time_t fatTime = AudioMoth_getTime();
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

    GPIO_PinModeSet(LED_GPIOPORT, RED_PIN, gpioModePushPull, state);

}

void AudioMoth_setBothLED(bool state) {

    GPIO_PinModeSet(LED_GPIOPORT, RED_PIN, gpioModePushPull, state);
    GPIO_PinModeSet(LED_GPIOPORT, GREEN_PIN, gpioModePushPull, state);

}

void AudioMoth_setGreenLED(bool state) {

    GPIO_PinModeSet(LED_GPIOPORT, GREEN_PIN, gpioModePushPull, state);

}

/* Functions to handle file system */

bool AudioMoth_enableFileSystem(void) {

    /* Turn SD card on*/

    GPIO_PinOutClear(SDEN_GPIOPORT, SD_ENABLE_N);

    /* Initialise MicroSD driver */

    MICROSD_Init();

    /* Check SD card status */

    DSTATUS resCard = disk_initialize(0);

    if (resCard == STA_NOINIT || resCard == STA_NODISK || resCard == STA_PROTECT) {
        return false;
    }

    /* Initialise file system */

    if (f_mount(0, &fatfs) != FR_OK) {
        return false;
    }

    return true;

}

void AudioMoth_disableFileSystem(void) {

    /* Turn SD card off*/

    GPIO_PinOutSet(SDEN_GPIOPORT, SD_ENABLE_N);

}

bool AudioMoth_openFile(char *filename) {

    /* Open a file for writing. Always overwrite an existing file with the same name */

    FRESULT res = f_open(&file, filename,  FA_CREATE_ALWAYS | FA_WRITE);

    if (res != FR_OK) {
        return false;
    }

    return true;

}

bool AudioMoth_appendFile(char *filename) {

    /* Open the file for writing. Append existing existing file with the same name */

    FRESULT res = f_open(&file, filename,  FA_OPEN_ALWAYS | FA_WRITE);

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

bool AudioMoth_closeFile(void) {

    FRESULT res = f_close(&file);

    if (res != FR_OK) {
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

/* Functions to enable and disable EBI */

static void enableEBI(void) {

  /* Enable clocks */

  CMU_ClockEnable(cmuClock_EBI, true);

  EBI_Init_TypeDef ebiConfig = EBI_INIT_DEFAULT;

  /* Configure GPIO pins as push pull */

  /* EBI AD0..07 data pins*/

  GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD00, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD01, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD02, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD03, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD04, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD05, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD06, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD07, gpioModePushPull, 0);

  /* EBI AD08..15 address pins*/

  GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD08, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD09, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD10, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD11, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD12, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD13, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD14, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD15, gpioModePushPull, 0);

  /* EBI A16..24 extension address pins*/

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

  /* EBI CS0-CS1 */

  GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL1, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL2, gpioModePushPull, 0);

  /* EBI WEN/OEN */

  GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_OE, gpioModePushPull, 0);
  GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_WE, gpioModePushPull, 0);

  /* Configure EBI controller, changing default values */

  ebiConfig.mode = ebiModeD8A8;
  ebiConfig.banks = EBI_BANK0;
  ebiConfig.csLines = EBI_CS0 | EBI_CS1;
  ebiConfig.readHalfRE = true;

  ebiConfig.aLow = ebiALowA8;
  ebiConfig.aHigh = ebiAHighA18;

  /* Address Setup and hold time */

  ebiConfig.addrHoldCycles  = 0;
  ebiConfig.addrSetupCycles = 0;

  /* Read cycle times */

  ebiConfig.readStrobeCycles = 3;
  ebiConfig.readHoldCycles   = 1;
  ebiConfig.readSetupCycles  = 2;

  /* Write cycle times */

  ebiConfig.writeStrobeCycles = 6;
  ebiConfig.writeHoldCycles   = 0;
  ebiConfig.writeSetupCycles  = 0;

  ebiConfig.location = ebiLocation1;

  /* Configure EBI */

  EBI_Init(&ebiConfig);

}

static void disableEBI(void) {

    /* Disable GPIO pins */

    /* EBI AD0..07 data pins*/

    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD00, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD01, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD02, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD03, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD04, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD05, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD06, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_AD07, gpioModeDisabled, 0);

    /* EBI AD08..15 address pins*/

    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD08, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD09, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD10, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD11, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD12, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD13, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD14, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD15, gpioModeDisabled, 0);

    /* EBI A16..24 extension address pins*/

    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A16, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A17, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A18, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A19, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A21, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A22, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A23, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A24, gpioModeDisabled, 0);

    /* EBI CS0-CS1 */

    GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL1, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL2, gpioModeDisabled, 0);

    /* EBI WEN/OEN */

    GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_OE, gpioModeDisabled, 0);
    GPIO_PinModeSet(EBI_GPIOPORT_F, EBI_WE, gpioModeDisabled, 0);

    /* Turn off EBI clock */

    CMU_ClockEnable(cmuClock_EBI, false);

}

/* Function configure GPIO pins */

static void setupGPIO(void) {

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
	GPIO_PinModeSet(VREF_GPIOPORT, VREF_EN, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortA, 12, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortA, 13, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortA, 14, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_A, EBI_AD08, gpioModeDisabled, 0);

	/* GPIO B */

	GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A16, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_B, EBI_A17, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 2, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 3, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 4, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 5, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 6, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 9, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 10, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 11, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortB, 12, gpioModeDisabled, 0);

	/* GPIO C */

	GPIO_PinModeSet(gpioPortC, 0, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortC, 1, gpioModeDisabled, 0);
	GPIO_PinModeSet(BAT_MON_GPIOPORT, BAT_MON_EN, gpioModePushPull, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A15, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A09, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_C, EBI_A10, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortC, 11, gpioModeDisabled, 0);

	/* GPIO D */

	GPIO_PinModeSet(gpioPortD, 1, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortD, 2, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortD, 3, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortD, 4, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortD, 5, gpioModeDisabled, 0);
	GPIO_PinModeSet(VMIC_GPIOPORT, VMIC_EN_N, gpioModePushPull, 1);
	GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL1, gpioModeDisabled, 0);
	GPIO_PinModeSet(EBI_GPIOPORT_D, EBI_CSEL2, gpioModeDisabled, 0);
	GPIO_PinModeSet(SRAMEN_GPIOPORT, SRAM_ENABLE_N, gpioModePushPull, 1);
	GPIO_PinModeSet(SDEN_GPIOPORT, SD_ENABLE_N, gpioModePushPull, 1);

	/* GPIO E */

	GPIO_PinModeSet(VMIC_GPIOPORT, VMIC_EN_N, gpioModePushPull, 1);
	GPIO_PinModeSet(EBI_GPIOPORT_E, EBI_A08, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortE, 2, gpioModeDisabled, 0);
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

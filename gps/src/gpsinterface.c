/****************************************************************************
 * gpsinterface.c
 * openacousticdevices.info
 * October 2022
 *****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_timer.h"

#include "gpsinterface.h"

/* Hardware constants */

#define UART_RX_GPIOPORT                        gpioPortB
#define UART_RX_PIN                             10

#define PPS_GPIOPORT                            gpioPortA
#define PPS_PIN                                 8

/* Timer constants */

#define TIMER_OVERFLOW_INTERVAL                 (1 << 16)

/* Tick counters */

static uint32_t currentTickCounter;

static uint32_t maximumTickCounter;

/* Counter parameters */

static uint32_t counterPeriod;

static uint32_t counterFrequency;

/* Interrupt handlers for RX */

void UART1_RX_IRQHandler() {

    /* Get the interrupt mask */

    uint32_t interruptMask = USART_IntGet(UART1);

    /* Clear the interrupt */

    USART_IntClear(UART1, interruptMask);

    /* Handle the interrupt */

    if ((interruptMask & UART_IF_RXDATAV) && (UART1->STATUS & UART_STATUS_RXDATAV)) {

        /* Read the received byte */

        uint8_t byte = USART_Rx(UART1);

        /* Call the interrupt handler */

        GPSInterface_handleReceivedByte(byte);

    }

}

/* Interrupt handlers for PPS */

void TIMER2_IRQHandler(void) {

    /* Get the interrupt mask */

    uint32_t interruptMask = TIMER_IntGet(TIMER2);

    /* Clear the interrupt */

    TIMER_IntClear(TIMER2, interruptMask);

    /* Handle the interrupt */

    if (interruptMask & TIMER_IF_CC0) {

        uint32_t counter = TIMER_CaptureGet(TIMER2, 0);

        GPSInterface_handlePulsePerSecond(counter, counterPeriod, counterFrequency);

    }

    if (interruptMask & TIMER_IF_OF) {

        currentTickCounter += 1;

        if (currentTickCounter > maximumTickCounter) {

            currentTickCounter = 0;

            GPSInterface_handleTick();

        }

    }

}

/* Public functions */

void GPSInterface_disable(void) {

    /* Disable the RX UART */

    NVIC_DisableIRQ(UART1_RX_IRQn);

    USART_Reset(UART1);

    CMU_ClockEnable(cmuClock_UART1, false);

    /* Disable the PPS timer and interrupt */

    NVIC_DisableIRQ(TIMER2_IRQn);

    TIMER_Reset(TIMER2);

    CMU_ClockEnable(cmuClock_TIMER2, false);

    /* Disable the pins */

    GPIO_PinModeSet(UART_RX_GPIOPORT, UART_RX_PIN, gpioModeDisabled, 0);

    GPIO_PinModeSet(PPS_GPIOPORT, PPS_PIN, gpioModeDisabled, 0);

}

void GPSInterface_disableTicks(void) {

    TIMER_IntDisable(TIMER2, TIMER_IF_OF);

}

void GPSInterface_enable(uint32_t ticksPerSecond) {

    /* Enable the RX pin */

    GPIO_PinModeSet(UART_RX_GPIOPORT, UART_RX_PIN, gpioModeInputPull, 1);

    /* Enable UART clock */

    CMU_ClockEnable(cmuClock_UART1, true);

    /* Initialise interface */

    USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

    uartInit.enable = usartDisable;

    uartInit.baudrate = 9600;

    USART_InitAsync(UART1, &uartInit);

    /* Prepare interrupts */

    USART_IntEnable(UART1, UART_IF_RXDATAV);

    NVIC_ClearPendingIRQ(UART1_RX_IRQn);

    NVIC_EnableIRQ(UART1_RX_IRQn);

    /* Enable route */

    UART1->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_LOCATION_LOC2;

    /* Enable UART */

    USART_Enable(UART1, usartEnableRx);

    /* Set up PPS pin and pull low */

    GPIO_PinModeSet(PPS_GPIOPORT, PPS_PIN, gpioModeInputPull, 0);

    /* Enable the ADC timer */

    CMU_ClockEnable(cmuClock_TIMER2, true);

    /* Set up tick overflow */

    currentTickCounter = 0;

    counterPeriod = TIMER_OVERFLOW_INTERVAL;

    counterFrequency = CMU_ClockFreqGet(cmuClock_TIMER2);

    maximumTickCounter = ticksPerSecond > 0 ? counterFrequency / TIMER_OVERFLOW_INTERVAL / ticksPerSecond : UINT32_MAX;

    TIMER_TopSet(TIMER2, counterPeriod - 1);

    /* Initialise the ADC timer capture interrupt */

    TIMER_InitCC_TypeDef timerCCInit = {
        .eventCtrl  = timerEventEveryEdge,
        .edge       = timerEdgeRising,
        .prsSel     = timerPRSSELCh0,
        .cufoa      = timerOutputActionNone,
        .cofoa      = timerOutputActionNone,
        .cmoa       = timerOutputActionNone,
        .mode       = timerCCModeCapture,
        .filter     = false,
        .prsInput   = false,
        .coist      = false,
        .outInvert  = false,
    };

    TIMER_InitCC(TIMER2, 0, &timerCCInit);

    /* Initialise the interrupts */

    if (ticksPerSecond > 0) TIMER_IntEnable(TIMER2, TIMER_IF_OF);

    TIMER_IntEnable(TIMER2, TIMER_IF_CC0);

    NVIC_ClearPendingIRQ(TIMER2_IRQn);

    NVIC_EnableIRQ(TIMER2_IRQn);

    /* Start the timer */

    TIMER_Enable(TIMER2, true);

}

/****************************************************************************
 * pinouts.h
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#ifndef __PINOUTS_H
#define __PINOUTS_H

/* Voltage reference power */

#define VREF_GPIOPORT                           gpioPortA
#define VREF_ENABLE                             11

/* Battery monitor */

#define BAT_MON_GPIOPORT                        gpioPortC
#define BAT_MON_ENABLE                          3

/* Red and green LED */

#define LED_GPIOPORT                            gpioPortC
#define GREEN_LED                               5
#define RED_LED                                 4

/* Switch 1 sense */

#define SWITCH_1_GPIOPORT                       gpioPortC
#define SWITCH_1_SENSE                          6

/* Switch 2 position */

#define SWITCH_2_GPIOPORT                       gpioPortD
#define SWITCH_2_SENSE                          8

/* SRAM power */

#define SRAMEN_GPIOPORT                         gpioPortD
#define SRAM_ENABLE_N                           11

/* SD card power */

#define SDEN_GPIOPORT                           gpioPortD
#define SD_ENABLE_N                             12

/* Internal microphone power */

#define VMIC_GPIOPORT                           gpioPortE
#define VMIC_ENABLE_N                           0

/* External microphone power */

#define JCK_GPIOPORT                            gpioPortE
#define JCK_ENABLE_N                            2

/* External microphone sense */

#define JCK_DETECT_GPIOPORT                     gpioPortA
#define JCK_DETECT                              14

/* Version control sense */

#define VERSION_CONTROL_GPIOPORT                gpioPortD
#define VERSION_CONTROL                         5

/* USB data */

#define USB_DATA_GPIOPORT                       gpioPortF
#define USB_P                                   11

/* EBI */

#define EBI_GPIOPORT_A                          gpioPortA
#define EBI_AD08                                15
#define EBI_AD09                                0
#define EBI_AD10                                1
#define EBI_AD11                                2
#define EBI_AD12                                3
#define EBI_AD13                                4
#define EBI_AD14                                5
#define EBI_AD15                                6

#define EBI_GPIOPORT_E                          gpioPortE
#define EBI_AD00                                8
#define EBI_AD01                                9
#define EBI_AD02                                10
#define EBI_AD03                                11
#define EBI_AD04                                12
#define EBI_AD05                                13
#define EBI_AD06                                14
#define EBI_AD07                                15
#define EBI_A08                                 1
#define EBI_A11                                 4
#define EBI_A12                                 5
#define EBI_A13                                 6
#define EBI_A14                                 7

#define EBI_GPIOPORT_B                          gpioPortB
#define EBI_A16                                 0
#define EBI_A17                                 1
#define EBI_A18                                 2
#define EBI_A19                                 3
#define EBI_A20                                 4
#define EBI_A21                                 5
#define EBI_A22                                 6

#define EBI_GPIOPORT_C                          gpioPortC
#define EBI_A15                                 8
#define EBI_A09                                 9
#define EBI_A10                                 10
#define EBI_A23                                 0
#define EBI_A24                                 1

#define EBI_GPIOPORT_D                          gpioPortD
#define EBI_CSEL1                               9
#define EBI_CSEL2                               10

#define EBI_GPIOPORT_F                          gpioPortF
#define EBI_OE                                  9
#define EBI_WE                                  8

#endif /* __PINOUTS_H */

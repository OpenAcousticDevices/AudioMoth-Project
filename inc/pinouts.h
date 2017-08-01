/****************************************************************************
 * pinouts.h
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#ifndef PINOUTS_H_
#define PINOUTS_H_

/* MIC VOLTAGE */

#define VREF_GPIOPORT                           gpioPortA
#define VREF_EN                                 11

/* BATTERY MONITORING */

#define BAT_MON_GPIOPORT                        gpioPortC
#define BAT_MON_EN                              3

/* LED GPIO */

#define LED_GPIOPORT                            gpioPortC
#define GREEN_PIN                               5
#define RED_PIN                                 4

/* Button GPIO */

#define SWITCH_GPIOPORT                         gpioPortC
#define SWITCH_PIN                              6

/* USB GPIO */

#define USB_GPIOPORT                            gpioPortD
#define USB_PIN                                 8

#define SRAMEN_GPIOPORT                         gpioPortD
#define SRAM_ENABLE_N                           11

/* Switch GPIO */

#define SDEN_GPIOPORT                           gpioPortD
#define SD_ENABLE_N                             12

/* MIC VOLTAGE */

#define VMIC_GPIOPORT                           gpioPortE
#define VMIC_EN_N                               0

/* MIC VOLTAGE */

#define USB_DATA_GPIOPORT                       gpioPortF
#define USB_P                                   11

/* EBI Pinouts*/

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

#endif /* PINOUTS_H_ */

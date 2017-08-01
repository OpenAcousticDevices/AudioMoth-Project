/****************************************************************************
 * usbdescriptors.h
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#ifndef __USBDESCRIPTORS_H
#define __USBDESCRIPTORS_H

#define USB_EP0_SIZE 64
#define USB_MAX_EP_SIZE 64

/* HID Report Descriptor */

SL_ALIGN(4)
static const char HID_ReportDescriptor[] __attribute__ ((aligned(4)))= {
    0x06, 0xAB, 0xFF,
    0x0A, 0x00, 0x02,
    0xA1, 0x01,         // Collection 0x01
    0x75, 0x08,         // report size = 8 bits
    0x15, 0x00,         // logical minimum = 0
    0x26, 0xFF, 0x00,   // logical maximum = 255
    0x95, 64,           // report count
    0x09, 0x01,         // usage
    0x81, 0x02,         // Input (array)
    0x95, 64,           // report count
    0x09, 0x02,         // usage
    0x91, 0x02,         // Output (array)
    0xC0                // end collection
};

/* HID Descriptor */

SL_ALIGN(4)
static const char HID_Descriptor[] __attribute__ ((aligned(4)))= {
    USB_HID_DESCSIZE,                     /* bLength               */
    USB_HID_DESCRIPTOR,                   /* bDescriptorType       */
    0x11,                                 /* bcdHID (LSB)          */
    0x01,                                 /* bcdHID (MSB)          */
    0,                                    /* bCountryCode          */
    1,                                    /* bNumDescriptors       */
    USB_HID_REPORT_DESCRIPTOR,            /* bDecriptorType        */
    sizeof(HID_ReportDescriptor),         /* wDescriptorLength(LSB)*/
    0                                     /* wDescriptorLength(MSB)*/
};

/* Device Descriptor */

SL_ALIGN(4)
static const USB_DeviceDescriptor_TypeDef deviceDesc __attribute__ ((aligned(4)))= {
    .bLength            = USB_DEVICE_DESCSIZE,            /* Size of the Descriptor in Bytes */
    .bDescriptorType    = USB_DEVICE_DESCRIPTOR,          /* Device Descriptor type */
    .bcdUSB             = 0x0200,                         /* USB 2.0 compliant */
    .bDeviceClass       = 0,                              /* Vendor unique device */
    .bDeviceSubClass    = 0,                              /* Ignored for vendor unique device */
    .bDeviceProtocol    = 0,                              /* Ignored for vendor unique device */
    .bMaxPacketSize0    = USB_EP0_SIZE,                   /* Max packet size for EP0 */
    .idVendor           = 0x10C4,                         /* VID */
    .idProduct          = 0x0002,                         /* PID */
    .bcdDevice          = 0x0000,                         /* Device Release number */
    .iManufacturer      = 1,                              /* Index of Manufacturer String Descriptor */
    .iProduct           = 2,                              /* Index of Product String Descriptor */
    .iSerialNumber      = 3,                              /* Index of Serial Number String Descriptor */
    .bNumConfigurations = 1                               /* Number of Possible Configurations */

};

/* Configuration Descriptor */

SL_ALIGN(4)
static const uint8_t configDesc[] __attribute__ ((aligned(4)))= {

    /*** Configuration descriptor ***/

    USB_CONFIG_DESCSIZE,                  /* bLength              */
    USB_CONFIG_DESCRIPTOR,                /* bDescriptorType      */

    USB_CONFIG_DESCSIZE +                 /* wTotalLength (LSB)   */
    USB_INTERFACE_DESCSIZE +
    USB_HID_DESCSIZE +
    (USB_ENDPOINT_DESCSIZE * NUM_EP_USED),

    (USB_CONFIG_DESCSIZE +                /* wTotalLength (MSB)   */
    USB_INTERFACE_DESCSIZE +
    USB_HID_DESCSIZE +
    (USB_ENDPOINT_DESCSIZE * NUM_EP_USED))>>8,

    1,                                    /* bNumInterfaces       */
    1,                                    /* bConfigurationValue  */
    0,                                    /* iConfiguration       */
    CONFIG_DESC_BM_RESERVED_D7 |          /* bmAttrib             */
    CONFIG_DESC_BM_SELFPOWERED |
    CONFIG_DESC_BM_REMOTEWAKEUP,
    CONFIG_DESC_MAXPOWER_mA(100),         /* bMaxPower: 100 mA    */

    /*** Interface descriptor ***/

    USB_INTERFACE_DESCSIZE,               /* bLength              */
    USB_INTERFACE_DESCRIPTOR,             /* bDescriptorType      */
    0,                                    /* bInterfaceNumber     */
    0,                                    /* bAlternateSetting    */
    NUM_EP_USED,                          /* bNumEndpoints        */
    0x03,                                 /* bInterfaceClass      */
    0,                                    /* bInterfaceSubClass   */
    0,                                    /* bInterfaceProtocol   */
    0,                                    /* iInterface           */

    /*** HID descriptor ***/

    USB_HID_DESCSIZE,                     /* bLength               */
    USB_HID_DESCRIPTOR,                   /* bDescriptorType       */
    0x11,                                 /* bcdHID (LSB)          */
    0x01,                                 /* bcdHID (MSB)          */
    0,                                    /* bCountryCode          */
    1,                                    /* bNumDescriptors       */
    USB_HID_REPORT_DESCRIPTOR,            /* bDecriptorType        */
    sizeof(HID_ReportDescriptor),         /* wDescriptorLength(LSB)*/
    0,                                    /* wDescriptorLength(MSB)*/

    /*** Bulk Endpoint Descriptor (OUT) ***/

    USB_ENDPOINT_DESCSIZE,                /* bLength              */
    USB_ENDPOINT_DESCRIPTOR,              /* bDescriptorType      */
    EP_OUT,                               /* bEndpointAddress     */
    USB_EPTYPE_INTR,                      /* bmAttributes         */
    USB_MAX_EP_SIZE,                      /* wMaxPacketSize (LSB) */
    0,                                    /* wMaxPacketSize (MSB) */
    1,                                    /* bInterval            */

    /*** Interrupt Endpoint Descriptor (IN) ***/

    USB_ENDPOINT_DESCSIZE,                /* bLength              */
    USB_ENDPOINT_DESCRIPTOR,              /* bDescriptorType      */
    EP_IN,                                /* bEndpointAddress     */
    USB_EPTYPE_INTR,                      /* bmAttributes         */
    USB_MAX_EP_SIZE,                      /* wMaxPacketSize (LSB) */
    0,                                    /* wMaxPacketSize (MSB) */
    1,                                    /* bInterval            */
};

/* String Descriptors */

STATIC_CONST_STRING_DESC_LANGID(langID, 0x04, 0x09);

STATIC_CONST_STRING_DESC(iManufacturer, 'o', 'p', 'e', 'n', 'a', 'c', 'o', 'u', 's', 't', 'i', 'c', 'd', 'e', 'v', 'i', 'c', 'e', 's', '.', 'i', 'n', 'f', 'o');

STATIC_CONST_STRING_DESC(iProduct, 'A','u','d','i','o','M','o','t','h');

STATIC_CONST_STRING_DESC(iSerialNumber, '0', '1','0','0');

/* Endpoint buffer sizes. Uses 1 for Control/Interrupt endpoints and 2 for Bulk endpoints */

static const uint8_t bufferingMultiplier[ NUM_EP_USED + 1 ] = {
    1,  /* Control */
    1,  /* Interrupt */
    1   /* Interrupt */
};

/* Define string array */

static const void * const strings[] = {
    &langID,
    &iManufacturer,
    &iProduct,
    &iSerialNumber
};

/* Define callbacks that are called by the USB stack on different events */

static const USBD_Callbacks_TypeDef callbacks = {
    .usbReset        = NULL,              /* Called whenever USB reset signalling is detected on the USB port. */
    .usbStateChange  = stateChange,       /* Called whenever the device change state.  */
    .setupCmd        = setupCmd,          /* Called on each setup request received from host. */
    .isSelfPowered   = NULL,              /* Called whenever the device stack needs to query if the device is currently self- or bus-powered. */
    .sofInt          = NULL               /* Called at each SOF (Start of Frame) interrupt. If NULL, the device stack will not enable the SOF interrupt. */
};

/* Structure passed to USBD_Init() in order to initialize the USB stack */

static const USBD_Init_TypeDef initstruct = {
    .deviceDescriptor    = &deviceDesc,
    .configDescriptor    = configDesc,
    .stringDescriptors   = strings,
    .numberOfStrings     = sizeof(strings)/sizeof(void*),
    .callbacks           = &callbacks,
    .bufferingMultiplier = bufferingMultiplier,
    .reserved            = 0
};

#endif /* __USBDESCRIPTORS_H */

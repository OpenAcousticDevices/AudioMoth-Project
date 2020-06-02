/****************************************************************************
 * usbcallbacks.h
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#ifndef __USBCALLBACKS_H
#define __USBCALLBACKS_H

/* Callback which provides the HID specific descriptors */

int setupCmd(const USB_Setup_TypeDef *setup);

/* Callback to start the USB reading process when the device is configured */

void stateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState);

/* Callback on completion of data send  on HID */

int dataSentHIDCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

/* Callback on completion of data send on Web USB */

int dataSentWebUSBCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

/* Callback on receipt of message from the USB host on HID */

int dataReceivedHIDCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

/* Callback on receipt of message from the USB host on Web USB */

int dataReceivedWebUSBCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

#endif /* __USBCALLBACKS_H */

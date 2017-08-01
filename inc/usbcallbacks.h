/****************************************************************************
 * usbcallbacks.h
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#ifndef _USBCALLBACKS_H_
#define _USBCALLBACKS_H_

/* Callback which provides the HID specific descriptors */

int setupCmd(const USB_Setup_TypeDef *setup);

/* Callback to start the USB reading process when the device is configured */

void stateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState);

/* Callback on completion of data send */

int dataSentCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

/* Callback on receipt of message from the USB host */

int dataReceivedCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

#endif /* _USBCALLBACKS_H_ */

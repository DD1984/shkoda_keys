#ifndef __USB_H__
#define __USB_H__

#include "usbd_customhid.h"

extern USBD_HandleTypeDef	USBD_Device;

void USB_Config(void);
int usb_send_msg(uint8_t *buf, uint32_t len);
void disconnect_usb(void);

#endif

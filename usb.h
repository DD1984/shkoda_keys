#ifndef __USB_H__
#define __USB_H__

extern USBD_HandleTypeDef	USBD_Device;

void USB_Config(void);
int usb_send_msg(uint8_t *buf, uint32_t len);

#endif

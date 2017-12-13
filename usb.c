#include "stm32f1xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"

USBD_HandleTypeDef	USBD_Device;

void USB_Config(void)
{
	USBD_Init(&USBD_Device, &HID_Desc, 0);
	USBD_RegisterClass(&USBD_Device, &USBD_CUSTOM_HID);
	USBD_CUSTOM_HID_RegisterInterface(&USBD_Device, &USBD_CustomHID_fops_FS);
	USBD_Start(&USBD_Device);
}

int usb_send_msg(uint8_t *buf, uint32_t len)
{
	USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef *)USBD_Device.pClassData;
	if (hhid->state != CUSTOM_HID_IDLE)
		return -1;

	USBD_CUSTOM_HID_SendReport(&USBD_Device, buf, len);
	return 0;
}

void disconnect_usb(void)
{
	//disconnect usb
	GPIO_InitTypeDef  GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

	HAL_Delay(50);

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
}

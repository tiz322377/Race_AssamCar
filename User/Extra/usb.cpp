//
// Created by Administrator on 25-3-2.
//
#ifdef STM32F103xB
#include "usb.hpp"


#include "usbd_core.h"
#include "usb_device.h"

extern "C" USBD_HandleTypeDef hUsbDeviceFS;

extern "C" void USBSerialInit() {
    __HAL_RCC_USB_FORCE_RESET();
    HAL_Delay(200);
    __HAL_RCC_USB_RELEASE_RESET();

    GPIO_InitTypeDef GPIO_InitStructure;
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStructure.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(300);
}

void Peripheral::USBSerial::Transmit(const uint8_t * _buffer, const uint16_t _size) {
    USBD_LL_Transmit(&hUsbDeviceFS,CDC_IN_EP,const_cast<uint8_t *>(_buffer),_size);
}

USBD_StatusTypeDef Peripheral::USBSerial::Receive(uint8_t *_buffer,const uint16_t _size,const uint32_t _delay) {
    const auto res = USBD_LL_PrepareReceive(&hUsbDeviceFS,CDC_OUT_EP,_buffer,_size);
    HAL_Delay(_delay);
    return res;
}
#endif
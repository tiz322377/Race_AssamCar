//
// Created by Administrator on 25-3-2.
//

#ifndef USB_HPP
#define USB_HPP

#ifdef STM32F103xB

#include <cstdint>
#include "usbd_cdc.h"

namespace Peripheral {

    class USBSerial {
    public:



        static void Transmit(const uint8_t *_buffer, uint16_t _size);

        static USBD_StatusTypeDef Receive(uint8_t * _buffer,uint16_t _size,uint32_t _delay);
    };

} // Peripheral

extern "C" void USBSerialInit();

#endif

#endif //USB_HPP

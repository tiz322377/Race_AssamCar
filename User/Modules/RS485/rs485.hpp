//
// Created by Administrator on 25-6-1.
//

#ifndef RS485_HPP
#define RS485_HPP

#include "Peripheral/Uart.hpp"
#include "Peripheral/GPIO.hpp"

namespace Modules {

    class RS485 {
    public:

        RS485() = delete;

        using UART = Peripheral::Uart<Peripheral::Normal>;

        using UARTIT = Peripheral::Uart<Peripheral::Interrupt>;

        using UARTDMA = Peripheral::Uart<Peripheral::DMA>;

        using CPin = Peripheral::GPIOPin<Peripheral::Output>;

        static void Send(const UART* _uart,const CPin * _cpin, const uint8_t *_data,uint16_t _size,uint16_t _time);

        static void SendIT(const UARTIT* _uart,const CPin * _cpin, const uint8_t *_data,uint16_t _size);

        static void SendDMA(const UARTDMA* _uart,const CPin * _cpin, const uint8_t *_data,uint16_t _size);

    };

} // Modules

#endif //RS485_HPP

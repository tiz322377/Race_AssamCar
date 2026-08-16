//
// Created by Administrator on 25-6-1.
//

#include "rs485.hpp"

void Modules::RS485::Send(const UART *_uart, const CPin *_cpin, const uint8_t *_data, const uint16_t _size,
                          const uint16_t _time) {
    const auto s = _cpin->Read();
    _cpin->Write(true);
    _uart->Send(_data, _size, _time);
    _cpin->Write(s);
}

void Modules::RS485::SendIT(const UARTIT *_uart, const CPin *_cpin, const uint8_t *_data, const uint16_t _size) {
    const auto s = _cpin->Read();
    _cpin->Write(true);
    _uart->SendIT(_data, _size);
    _cpin->Write(s);
}

void Modules::RS485::SendDMA(const UARTDMA *_uart, const CPin *_cpin, const uint8_t *_data, const uint16_t _size) {
    const auto s = _cpin->Read();
    _cpin->Write(true);
    _uart->Send(_data, _size);
    _cpin->Write(s);
}

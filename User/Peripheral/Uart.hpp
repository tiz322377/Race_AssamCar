//
// Created by Administrator on 24-7-26.
//

#ifndef UART_HPP
#define UART_HPP

#include "Config/Config.hpp"
#include "Mode.hpp"

namespace Peripheral {
#ifdef HAL_UART_MODULE_ENABLED
    template<UPMode _mode>
    struct Uart;

    template<>
    struct Uart<Normal> {
        UART_HandleTypeDef *handle;

        explicit Uart(UART_HandleTypeDef *_handle): handle(_handle) {
        }

        void Send(const uint8_t *_buffer, const uint16_t _size, const uint32_t _waitTime = 100) const {
            HAL_UART_Transmit(handle, _buffer, _size, _waitTime);
        }

        void Send(const char *_buffer, const uint16_t _size, const uint32_t _waitTime = 100) const {
            HAL_UART_Transmit(handle, (uint8_t *) _buffer, _size, _waitTime);
        }

        void Receive(uint8_t *_buffer, const uint16_t _size, const uint32_t _waitTime = 100) const {
            HAL_UART_Receive(handle, _buffer, _size, _waitTime);
        }
    };

    template<>
    struct Uart<Interrupt> : public Uart<Normal> {
        explicit Uart(UART_HandleTypeDef *_handle): Uart<Normal>(_handle) {
        }

        void SendIT(const uint8_t *_buffer, const uint16_t _size) const {
            HAL_UART_Transmit_IT(handle, _buffer, _size);
        }

        void SendIT(const char *_buffer, const uint16_t _size) const {
            HAL_UART_Transmit_IT(handle, (uint8_t *) _buffer, _size);
        }

        void ReceiveIT(uint8_t *_buffer, const uint16_t _size) const {
            HAL_UART_Receive_IT(handle, _buffer, _size);
        }
        void ReceiveIdleIT(uint8_t *_buffer, const uint16_t _size) const {
            HAL_UARTEx_ReceiveToIdle_IT(handle, _buffer, _size);
        }
        void AbortReceiveIT() const {
            HAL_UART_AbortReceive_IT(handle);
        }
    };

    template<>
    struct Uart<DMA> : public Uart<Interrupt> {
        explicit Uart(UART_HandleTypeDef *_port): Uart<Interrupt>(_port) {
        }
        void SendDMA(const uint8_t *_buffer, const uint16_t _size) const {
            HAL_UART_Transmit_DMA(handle, _buffer, _size);
        }
        void SendDMA(const char *_buffer, const uint16_t _size) const {
            HAL_UART_Transmit_DMA(handle, (uint8_t *) _buffer, _size);
        }
        void ReceiveDMA(uint8_t *_buffer, const uint16_t _size) const {
            HAL_UART_Receive_DMA(handle, _buffer, _size);
        }
        void ReceiveIdleDMA(uint8_t *_buffer, const uint16_t _size = 32) const {
            HAL_UARTEx_ReceiveToIdle_DMA(handle, _buffer, _size);
        }
    };
#endif
} // Peripheral


#endif //UART_HPP

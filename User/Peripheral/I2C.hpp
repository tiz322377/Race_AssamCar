//
// Created by Administrator on 24-7-26.
//

#ifndef I2C_HPP
#define I2C_HPP
#ifdef HAL_I2C_MODULE_ENABLE
#include "Config/Config.hpp"

namespace Peripheral {

    enum I2CMode {
        Master,
        Slave
    };

    enum class I2CWorkMode {
        Normal,
        Interrupt,
        DMA
    };

    template<I2CMode _mode, I2CWorkMode _workMode>
    class I2C;

    template<>
    class I2C<Master,I2CWorkMode::DMA> {
    public:
        I2C_HandleTypeDef * const Port;
        explicit I2C(I2C_HandleTypeDef * const _port) : Port(_port)  {}
        void Send(uint8_t * _buffer,const uint8_t _addr,const uint16_t _size) const {
            HAL_I2C_Master_Transmit_DMA(Port, _addr,_buffer,_size);
        }
        void Receive(uint8_t * _buffer,const uint8_t _addr,const uint16_t _size) const {
            HAL_I2C_Master_Receive_DMA(Port, _addr,_buffer,_size);
        }
    };

    template<>
    class I2C<Master,I2CWorkMode::Normal> {
    public:
        I2C_HandleTypeDef * const Port;

        explicit I2C(I2C_HandleTypeDef * const _port) : Port(_port)  {

        }
        void Send(uint8_t * _buffer,const uint8_t _addr,const uint16_t _size,uint16_t _time = 100) const {
            HAL_I2C_Master_Transmit(Port, _addr,_buffer,_size,_time);
        }
        void Receive(uint8_t * _buffer,const uint8_t _addr,const uint16_t _size,uint16_t _time = 100) const {
            HAL_I2C_Master_Receive(Port, _addr,_buffer,_size,_time);
        }
        void MemWrite(
                uint16_t _devAddr, uint16_t _memAddr, uint16_t _addrSize,
            uint8_t *pData, uint16_t size, uint32_t _time = 100) const {
            HAL_I2C_Mem_Write(Port, _devAddr, _memAddr, _addrSize, pData, size, _time);
        }

        void MemRead(
            uint16_t _devAddr, uint16_t _memAddr, uint16_t _addrSize,
            uint8_t *pData, uint16_t size, uint32_t _time = 100) const
        {
            HAL_I2C_Mem_Read(Port, _devAddr, _memAddr, _addrSize, pData, size, _time);
        }


    };


}// Peripheral
#endif
#endif //I2C_HPP

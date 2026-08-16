//
// Created by Administrator on 25-3-2.
//



#ifndef FDCAN_HPP
#define FDCAN_HPP

#include "Config/Config.hpp"

#ifdef HAL_FDCAN_MODULE_ENABLED

namespace Peripheral {

    enum CANType {
        Classic,
        FDCAN
    };

    template<CANType _type>
    class FDCan;

    template<>
    class FDCan<Classic> {

        FDCAN_HandleTypeDef * fdcan;

    public:

        explicit FDCan(FDCAN_HandleTypeDef * _fdcan) : fdcan(_fdcan) {
            HAL_FDCAN_Start(fdcan);
        }

        void Transmit(uint8_t _addr, const uint8_t *_data, uint16_t _len) const;

        FDCAN_RxHeaderTypeDef Receive(uint8_t *_data) const;

        bool Verify() const;

    };

}

#endif


#endif //FDCAN_HPP

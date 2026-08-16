//
// Created by Administrator on 24-7-26.
//
//
#include "GPIO.hpp"
#ifdef HAL_GPIO_MODULE_ENABLED

namespace Peripheral {
    void GPIOPin<Output>::Write(bool _value) const {
        if (_value) {
            HAL_GPIO_WritePin(inf.port,inf.pin,GPIO_PIN_SET);
        }
        else {
            HAL_GPIO_WritePin(inf.port,inf.pin,GPIO_PIN_RESET);
        }

    }
}

#endif





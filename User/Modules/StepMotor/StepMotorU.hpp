//
// Created by Administrator on 24-10-18.
//

#ifndef LV_Reserve_STEPMOTORU_HPP
#define LV_Reserve_STEPMOTORU_HPP

#include "Peripheral/TIM.hpp"
#include "Peripheral/GPIO.hpp"

namespace Modules {

    struct StepMotorConfig {
        uint8_t cent;
    };

    enum StepMotorType {
        CANBus,
        Pluses,
        Serial,
        PlusesIO
    };

    template<StepMotorType _type>
    class StepMotor;
}

#endif //LV_Reserve_STEPMOTORU_HPP

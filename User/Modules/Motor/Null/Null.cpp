//
// Created by Administrator on 24-10-12.
//

#include "Null.hpp"

namespace Modules::Motor {

    Motor<Null,0,0,0>::Motor(
            const Peripheral::PwmChannel<Peripheral::Normal> &_pwmChannel,
            const Peripheral::GPIOPin<Peripheral::Output> &_o1Pin,
            const Peripheral::GPIOPin<Peripheral::Output> &_o2Pin,
            const Peripheral::EncoderPair &_encoderPair
    ) : pwmChannel(_pwmChannel), o1Pin(_o1Pin), o2Pin(_o2Pin), encoderPair(_encoderPair) {
        maxSpeed = 1.0f;
    }
}
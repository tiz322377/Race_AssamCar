//
// Created by Administrator on 24-10-12.
//

#ifndef LV_Reserve_CHASSISU_HPP
#define LV_Reserve_CHASSISU_HPP

#include "Modules/Motor/MotorU.hpp"

#include <cstdint>

namespace Platform::Chassis {
    enum MoveDirection {
        Forward = 1,
        Backward = 2,
        Left = 3,
        Right = 4,
        Rotate = 5,
        Lock = 6,
        LeftForward = 7,
        RightForward = 8,
        LeftBackward = 9,
        RightBackward = 10,
    };
}
#endif //LV_Reserve_CHASSISU_HPP

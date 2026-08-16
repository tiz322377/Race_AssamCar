//
// Created by Administrator on 24-10-12.
//

#ifndef LV_Reserve_MOTORU_HPP
#define LV_Reserve_MOTORU_HPP

#include <cstdint>

namespace Modules::Motor {

    struct MotorParas {
        const uint32_t MaxRPM;
        const uint32_t Encoder;
        const double Radius;
    };

    enum MotorType {
        Encode,
        Null
    };

    enum MotorDirection {
        Lock = 2,
        Ahead = 1,
        Back = 0
    };

    using uint = uint32_t;

    template<MotorType _type = Null,uint _rsCount = 0,uint _rdCount = 0,uint _rtCount = 0>
    class Motor;

}

#endif //LV_Reserve_MOTORU_HPP

//
// Created by Administrator on 24-10-23.
//

#ifndef LV_Reserve_DELAY_HPP
#define LV_Reserve_DELAY_HPP

#include "Config/Config.hpp"

#define DELAY 168

namespace Peripheral {

    class Delay {
    public:
        static void delayUs(uint32_t _delay);

        static void HighResDelay(double _time);
    };

} // Peripheral

#endif //LV_Reserve_DELAY_HPP

//
// Created by Administrator on 24-10-23.
//

#include "Delay.hpp"

namespace Peripheral {

    void Delay::HighResDelay(const double _time) {
        if (_time < 0)
            return;
        const auto ms = static_cast<uint32_t>(_time);
        const auto us = static_cast<uint32_t>((_time - static_cast<double >(ms)) * 1000);
        if (ms >0)
            for (int i = 0;i<ms;i++)
                delayUs(1000);
        if (us > 0)
            delayUs(us);
    }

    void Delay::delayUs(volatile uint32_t _delay) {
        static constexpr uint32_t cpuFrequencyMhz =  DELAY;	// STM32时钟主频
        int val;

        while (_delay != 0) {
            const int temp = _delay > 900 ? 900 : _delay;
            const int last = SysTick->VAL;
            if (int curr = last - cpuFrequencyMhz * temp; curr >= 0)
            {
                do
                {
                    val = SysTick->VAL;
                }
                while ((val < last) && (val >= curr));
            }
            else
            {
                curr += cpuFrequencyMhz * 1000;
                do
                {
                    val = SysTick->VAL;
                }
                while ((val <= last) || (val > curr));
            }
            _delay -= temp;
        }
    }

} // Peripheral
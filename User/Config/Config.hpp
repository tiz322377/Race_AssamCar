//
// Created by Administrator on 25-3-2.
//

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "main.h"

#if CMSIS_RTOS
#include "cmsis_os2.h"
#endif

inline void Delay(uint32_t _ticks) {
#if CMSIS_RTOS
    osDelay(_ticks);
#else
    HAL_Delay(_ticks);
#endif
}


namespace Config {

    enum TimeResult {
        TimeOut = 0,
        Success = 1,
        Error   = 2
    };

}

#endif //CONFIG_HPP

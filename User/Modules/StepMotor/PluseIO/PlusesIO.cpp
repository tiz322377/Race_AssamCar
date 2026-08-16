//
// Created by Administrator on 24-10-18.
//


#include "PlusesIO.hpp"

#ifdef CMSIS_RTOS
#include "cmsis_os2.h"
#endif

static inline void delay(uint32_t _delay) {
#if CMSIS_RTOS
    osDelay(_delay);
#else
    HAL_Delay(_delay);
#endif
}

namespace Modules {

    void StepMotor<PlusesIO>::StepOnce(const uint32_t _delay) const {
        step.Write(true);
        delay(_delay);
        step.Write(false);
    }

    void StepMotor<PlusesIO>::Steps(const uint32_t _steps, const uint32_t _delay) const {
        for (uint32_t i = 0; i < _steps; i++) {
            StepOnce(_delay);
        }
    }
}
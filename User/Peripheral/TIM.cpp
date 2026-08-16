//
// Created by Administrator on 24-7-26.
//
#include "TIM.hpp"

#ifdef HAL_TIM_MODULE_ENABLED

void Peripheral::PwmChannel<Peripheral::Normal>::SetDutyCycle(float _dutyCycle) const {
    if (_dutyCycle > 1.0f) _dutyCycle = 1.0f;
    if (_dutyCycle < 0.0f) _dutyCycle = 0.0f;
    const auto compare = handle->Instance->ARR;
    switch (channel) {
        case TIM_CHANNEL_1:
            handle->Instance->CCR1 = static_cast<uint32_t>(_dutyCycle * compare);
            break;
        case TIM_CHANNEL_2:
            handle->Instance->CCR2 = static_cast<uint32_t>(_dutyCycle * compare);
            break;
        case TIM_CHANNEL_3:
            handle->Instance->CCR3 = static_cast<uint32_t>(_dutyCycle * compare);
            break;
        case TIM_CHANNEL_4:
            handle->Instance->CCR4 = static_cast<uint32_t>(_dutyCycle * compare);
            break;
        default:
            break;
    }
}

void Peripheral::PwmChannel<Peripheral::Normal>::SetCompare(const uint32_t _compare) const {
    if (_compare > handle->Instance->ARR) {
        return; // 超出范围
    }
    switch (channel) {
        case TIM_CHANNEL_1:
            handle->Instance->CCR1 = _compare;
            break;
        case TIM_CHANNEL_2:
            handle->Instance->CCR2 = _compare;
            break;
        case TIM_CHANNEL_3:
            handle->Instance->CCR3 = _compare;
            break;
        case TIM_CHANNEL_4:
            handle->Instance->CCR4 = _compare;
            break;
        default:
            break;
    }
}

uint32_t Peripheral::PwmChannel<Peripheral::Normal>::GetCompare() const {
    switch (channel) {
        case TIM_CHANNEL_1:
            return handle->Instance->CCR1;
        case TIM_CHANNEL_2:
            return handle->Instance->CCR2;
        case TIM_CHANNEL_3:
            return handle->Instance->CCR3;
        case TIM_CHANNEL_4:
            return handle->Instance->CCR4;
        default:
            break;
    }
		return 0;
}

#endif

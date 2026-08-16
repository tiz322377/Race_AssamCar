//
// Created by Administrator on 24-7-26.
//

#ifndef PWMCHANNEL_HPP
#define PWMCHANNEL_HPP

#include "Config/Config.hpp"
#include "Mode.hpp"
#include <cstdint>

namespace Peripheral {

#ifdef HAL_TIM_MODULE_ENABLED

    template<UPMode _mode>
    struct Timer;

    template <>
    struct Timer<Normal> {
        TIM_HandleTypeDef * const handle;
        explicit Timer(TIM_HandleTypeDef * const _timer): handle(_timer) {
            
        }
        void Start() const {
            HAL_TIM_Base_Start(handle);
        }
        void Stop() const {
            HAL_TIM_Base_Stop(handle);
        }
        int16_t GetCount() const {
            return static_cast<int16_t>(handle->Instance->CNT);
        }
        void SetCount(int16_t _count) const {
            handle->Instance->CNT = static_cast<uint32_t>(_count);
        }
        void SetPeriod(uint32_t _period) const {
            handle->Instance->ARR = _period;
        }
        [[nodiscard]] uint32_t GetPeriod() const {
            return handle->Instance->ARR;
        } 
    };


    template <>
    struct Timer<Interrupt> : Timer<Normal> {
        explicit Timer(TIM_HandleTypeDef * const _timer): Timer<Normal>(_timer) {
        }
        void StartIT() const {
            HAL_TIM_Base_Start_IT(Timer<Normal>::handle);
        }
        void StopIT() const {
            HAL_TIM_Base_Stop_IT(Timer<Normal>::handle);
        }
    };

    template <UPMode _mode>
    struct PwmChannel;

    template <>
    struct PwmChannel<Normal> {
        TIM_HandleTypeDef * const handle;
        const uint32_t channel;
    
        explicit PwmChannel(TIM_HandleTypeDef * const _timer,const uint32_t _channel): handle(_timer), channel(_channel) {};
        void SetDutyCycle(float _dutyCycle) const;
        [[nodiscard]] float GetDutyCycle() const {
            return static_cast<float>(handle->Instance->CCR1) / static_cast<float>(handle->Instance->ARR);
        }
        void Start() const {
            HAL_TIM_PWM_Start(handle, channel);
        }
        void Stop() const {
            HAL_TIM_PWM_Stop(handle, channel);
        }
        void SetCompare(uint32_t _compare) const;
        [[nodiscard]] uint32_t GetCompare() const;
    };

    template <>
    struct PwmChannel<Interrupt> : PwmChannel<Normal> {
        explicit PwmChannel(TIM_HandleTypeDef * const _timer,const uint32_t _channel): PwmChannel<Normal>(_timer, _channel) {};
        void StartIT() const {
            HAL_TIM_PWM_Start_IT(handle, channel);
        }
        void StopIT() const {
            HAL_TIM_PWM_Stop_IT(handle, channel);
        }
    };

    template <>
    struct PwmChannel<DMA> : PwmChannel<Normal> {
        explicit PwmChannel(TIM_HandleTypeDef * const _timer,const uint32_t _channel): PwmChannel<Normal>(_timer, _channel) {};
        void StartDMA(uint32_t * _addr,uint32_t _length) const {
            HAL_TIM_PWM_Start_DMA(handle, channel, _addr, _length);
        }
        void StopDMA() const {
            HAL_TIM_PWM_Stop_DMA(handle, channel);
        }
    };

    struct EncoderPair {
        TIM_HandleTypeDef * const handle;
        const uint32_t channel1;
        const uint32_t channel2;
        EncoderPair(TIM_HandleTypeDef * const _timer,const uint32_t _channel1,const uint32_t _channel2) :
            handle(_timer), channel1(_channel1), channel2(_channel2) {}
        void Start() const {
            HAL_TIM_Encoder_Start(handle, channel1);
            HAL_TIM_Encoder_Start(handle, channel2);
        }
        void Stop() const {
            HAL_TIM_Encoder_Stop(handle, channel1);
            HAL_TIM_Encoder_Stop(handle, channel2);
        }
        [[nodiscard]] short GetCount() const {
            const auto tmp = handle->Instance->CNT;
            CounterClear();
            return static_cast<short>(tmp);
        }
    private:
        void CounterClear() const {
            handle->Instance->CNT = 0;
        }
    };
#endif
} // Peripheral


#endif //PWMCHANNEL_HPP

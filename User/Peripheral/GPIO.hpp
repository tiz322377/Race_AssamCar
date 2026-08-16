//
// Created by Administrator on 24-7-26.
//

#ifndef GPIO_HPP
#define GPIO_HPP

#include "Config/Config.hpp"

#ifdef HAL_GPIO_MODULE_ENABLED
namespace Peripheral {

    enum GPIOMode {
        Output,Input,Exit
    };

    template<GPIOMode _mode>
    struct GPIOPin;

    struct GPIOInf {
        GPIOInf (GPIO_TypeDef * _port,const uint16_t _pin) : port(_port),pin(_pin) {}
        GPIO_TypeDef * const port;
        const uint16_t pin;
    };

    template<>
    struct GPIOPin<Output> {

        explicit GPIOPin(GPIO_TypeDef * const _port,const uint16_t _pin) : inf(_port,_pin) {}
        void Write(bool _value) const ;
        [[nodiscard]] GPIO_PinState Read() const {
            return HAL_GPIO_ReadPin(inf.port,inf.pin);
        };
        void Toggle() const {
            HAL_GPIO_TogglePin(inf.port,inf.pin);
        }
        void operator = (const bool _value) const {
            Write(_value);
        }

        void Blink(const uint16_t _delay) const {
            Toggle();
            HAL_Delay(_delay);
            Toggle();
        }

        GPIOPin & operator = (const GPIOPin & _pin) = delete;
    private:
        const GPIOInf inf;
    };




    template<>
    struct GPIOPin<Input> {

        GPIOPin(GPIO_TypeDef * _port,const uint16_t _pin,const GPIO_PinState _defautlValue) : inf(_port,_pin),defautlValue(_defautlValue) {}
        [[nodiscard]] bool Read() const {
            return defautlValue != HAL_GPIO_ReadPin(inf.port,inf.pin);
        }
        bool operator == (const bool _value) const {
            return Read() == _value;
        }
    private:
        const GPIOInf inf;
        const GPIO_PinState defautlValue;
    };

    template<>
    struct GPIOPin<Exit> {
        GPIOInf inf;
        GPIOPin(GPIO_TypeDef * _port,const uint16_t _pin) : inf(_port,_pin) {}
    };

} // Peripheral

#endif
#endif //GPIO_HPP

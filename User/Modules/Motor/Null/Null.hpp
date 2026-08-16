//
// Created by Administrator on 24-10-12.
//

#ifndef LV_Reserve_NULL_HPP
#define LV_Reserve_NULL_HPP

#include "Modules/Motor/MotorU.hpp"
#include "Peripheral/TIM.hpp"
#include "Peripheral/GPIO.hpp"

template<>
class Modules::Motor::Motor<Modules::Motor::Null,0,0,0> {
    const Peripheral::PwmChannel<Peripheral::Normal> & pwmChannel;
    const Peripheral::GPIOPin<Peripheral::Output> & o1Pin;
    const Peripheral::GPIOPin<Peripheral::Output> & o2Pin;
    const Peripheral::EncoderPair & encoderPair;
    float maxSpeed;
public:
    explicit Motor(
        const Peripheral::PwmChannel<Peripheral::Normal> & _pwmChannel,
        const Peripheral::GPIOPin<Peripheral::Output> & _o1Pin,
        const Peripheral::GPIOPin<Peripheral::Output> & _o2Pin,
        const Peripheral::EncoderPair & _encoderPair
    );
    void SetDirection(const bool _dir) const {
        o1Pin.Write(_dir);
        o2Pin.Write(!_dir);
    }
    void SetMaxSpeed(const float _speed) {
        maxSpeed = 1.0f;
        if (_speed > 1.0f) {
        }
        if (_speed < 0.0f) {
            maxSpeed = 0.0f;
        }
        else {
            maxSpeed = _speed;
        }
    }
    void SetSpeed(const float _speed) const {
        if (_speed > maxSpeed) {
            pwmChannel.SetDutyCycle(maxSpeed);
        }
        else {
            pwmChannel.SetDutyCycle(_speed);
        }
    }
};

#endif //LV_Reserve_NULL_HPP

//
// Created by Administrator on 24-7-27.
//

#ifndef MOTOR_HPP
#define MOTOR_HPP



#include "Peripheral/GPIO.hpp"
#include "Peripheral/TIM.hpp"
#include "Modules/Motor/MotorU.hpp"
#include "Modules/Motor/Pid.hpp"
#include <cstdint>

using uint = uint32_t;
template<uint _rsCount,uint _rdCount,uint _rtCount>
class Modules::Motor::Motor<Modules::Motor::Encode,_rsCount,_rdCount,_rtCount> {

    Pid<Single> sPidDatas[_rsCount];
    Pid<Double> dPidDatas[_rdCount];
    Pid<Triple> tPidDatas[_rtCount];

    const Peripheral::PwmChannel<Peripheral::Normal> pwmChannel;
    const Peripheral::GPIOPin<Peripheral::Output> o1Pin;
    const Peripheral::GPIOPin<Peripheral::Output> o2Pin;
    const Peripheral::EncoderPair encoderPair;

    MotorDirection state;

    float maxSpeed;

public:

    explicit Motor( Pid<Single> * spids,Pid<Double> * dpids,Pid<Triple> * tpids,
                    const Peripheral::PwmChannel<Peripheral::Normal> & _pwmChannel,
                    const Peripheral::GPIOPin<Peripheral::Output> & _o1Pin,
                    const Peripheral::GPIOPin<Peripheral::Output> & _o2Pin,
                    const Peripheral::EncoderPair & _encoderPair
    );

    float PidSUpdate(uint32_t _time,int16_t _count,uint _index);

    float PidDUpdate(uint32_t _time,int16_t _count,uint _index);

    float PidTUpdate(uint32_t _time,int16_t _count,float _dutyCycle,uint _index);

    void SetDirection(const bool _dir) const {
        o1Pin.Write(_dir);
        o2Pin.Write(!_dir);
    }

    void SetMaxSpeed(const float _speed) {
        if (_speed > 1.0f) {
            maxSpeed = 1.0f;
        }
        if (_speed < 0.0f) {
            maxSpeed = 0.0f;
        }
        else {
            maxSpeed = _speed;
        }
    }

    void SetSpeed (const float _speed) const {
        if (_speed > maxSpeed) {
            pwmChannel.SetDutyCycle(maxSpeed);
        }
        if (_speed < 0.0f) {
            pwmChannel.SetDutyCycle(_speed);
        }
    }

};// Modules

#endif //MOTOR_HPP

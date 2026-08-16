//
// Created by Administrator on 24-7-27.
//

#include "Encode.hpp"
#include "Peripheral/Mode.hpp"

namespace Modules::Motor {

    template<uint rsCount, uint rdCount, uint rtCount>
    Motor<Encode, rsCount, rdCount, rtCount>::Motor( Pid<Single> * spids,Pid<Double> * dpids, Pid<Triple> * tpids,
                    const Peripheral::PwmChannel<Peripheral::Normal> & _pwmChannel,
                    const Peripheral::GPIOPin<Peripheral::Output> & _o1Pin,
                    const Peripheral::GPIOPin<Peripheral::Output> & _o2Pin,
                    const Peripheral::EncoderPair & _encoderPair
    ): pwmChannel(_pwmChannel),o1Pin(_o1Pin),o2Pin(_o2Pin),encoderPair(_encoderPair) {
        for (uint i = 0; i <rsCount; i++) {
            sPidDatas[i] = spids[i];
        }
        for (uint i = 0; i < rdCount; i++) {
            dPidDatas[i] = dpids[i];
        }
        for (uint i = 0; i < rtCount; i++) {
            tPidDatas[i] = tpids[i];
        }
        maxSpeed = 1.0f;
    }

    template<uint rsCount, uint rdCount, uint rtCount>
    float Motor<Encode, rsCount, rdCount, rtCount>::PidSUpdate
        (const uint32_t _time, const int16_t _count,const uint _index)
    {
        auto temp = sPidDatas[_index].Update(_time,_count);
        if (temp > 0) {
            SetDirection(true);
            state = Ahead;
        }
        else {
            temp = -temp;
            SetDirection(false);
            state = Back;
        }
        if (temp >= 0.0f && temp <= 0.08f) {
            temp = 0.0f;
            sPidDatas[_index].Reset();
        }
        return temp;

    }

    template<uint rsCount, uint rdCount, uint rtCount>
    float Motor<Encode, rsCount, rdCount, rtCount>::PidDUpdate
            (const uint32_t _time, const int16_t _count,const uint _index) {
        auto temp = dPidDatas[_index].Update(_time,_count);
        if (temp > 0) {
            SetDirection(true);
            state = Ahead;
        }
        else {
            temp = -temp;
            SetDirection(false);
            state = Back;
        }
        return temp;
    }

    template<uint rsCount, uint rdCount, uint rtCount>
    float Motor<Encode, rsCount, rdCount, rtCount>::PidTUpdate
    (uint32_t _time, int16_t _count, float _dutyCycle,uint _index){
        auto temp = tPidDatas[_index].Update(_time,_dutyCycle,_count);
        if (temp > 0) {
            SetDirection(true);
            state = Ahead;
        }
        else {
            temp = -temp;
            SetDirection(false);
            state = Back;
        }
        return temp;
    }
}

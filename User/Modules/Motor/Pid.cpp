//
// Created by Administrator on 24-10-12.
//

#include "Pid.hpp"

namespace Modules::Motor {

    using i16 = int16_t;

    float Pid<Single>::Update(const i16 _change,const uint32_t _time) {
#define ToFloat static_cast<float>
        Target -= _change;

        integral += Target;

        if (integral > IntegralLimit) {
            integral = IntegralLimit;
        }
        if (integral < -IntegralLimit) {
            integral = -IntegralLimit;
        }

        auto temp = cpara.Kp * ToFloat(Target)
                    + cpara.Ki * ToFloat(integral)
                    - cpara.Kd * ToFloat(Target - lastError);


        temp =  temp / ToFloat(motorParas.MaxRPM) / 60.0f / ToFloat(_time) * 1000;



        lastError = Target;

        return temp;
    }

    void Pid<Single>::Reset() {
        lastError = 0;
        integral = 0;
        Target = 0;
    }

    void Pid<Double>::Reset() {
        Target = 0;
        clastError = 0;
        cintegral = 0;

        vTarget = 0;
        vlastError = 0;
        vintegral = 0;
    }

   void Pid<Triple>::Reset() {
        tError = 0;
        tLastError = 0;
        tIntegral = 0;

        cError = 0;
        cLastError = 0;
        cIntegral = 0;

        sError = 0;
        sLastError = 0;
        sIntegral = 0;
    }

    float Pid<Triple>::Update(uint16_t _change,float _dutyCycle,uint32_t _time) {
        auto time = ToFloat(_time);
        cError -= _change;
        cLastError = cError;
        cIntegral += cError;
        cIntegral %= cIntegralLimit;

        auto temp = cPara.Kp * ToFloat(cError)
                + cPara.Ki * ToFloat(cIntegral)
                - cPara.Kd * ToFloat(cError - cLastError);

        tError = temp - _dutyCycle;
        tLastError = tError;
        tIntegral += tError;
        tIntegral += ToFloat(250) * 60.0f / time / 1000.0f;

        if (tIntegral > tIntegralLimit) {
            tIntegral = tIntegralLimit;
        }

        uint16_t nowSpeed = _change /  _time + _change/ _time;
        nowSpeed /= 2.0f * mParas.MaxRPM ;
        sError = temp - ToFloat(nowSpeed);
        sLastError = sError;
        sIntegral += sError;
        sIntegral += ToFloat(250) * 60.0f / time / 1000.0f;

        if (sIntegral > sIntegralLimit) {
            sIntegral = sIntegralLimit;
        }

        temp = tPara.Kp * ToFloat(tError)
                + tPara.Ki * ToFloat(tIntegral)
                - tPara.Kd * ToFloat(tError - tLastError);
        return temp;
    }

    float Pid<Double>::Update(i16 _change, uint32_t _time) {
        Target -= _change;
        cintegral += Target;
        if (cintegral > IntegralLimit) {
            cintegral = IntegralLimit;
        }
        else if (cintegral < -IntegralLimit) {
            cintegral = -IntegralLimit;
        }
        auto vlTarget = (
                cpara.Kp * ToFloat(Target) +
                cpara.Ki * ToFloat(cintegral) -
                cpara.Kd * ToFloat(Target - clastError)/ToFloat(_time)
                )
                        / ToFloat(_time);
        vlTarget = vlTarget / ToFloat(motorParas.MaxRPM) / 60.0f * 1000.0f / 5.0f;
        //
        clastError = Target;
        auto limit = ToFloat(motorParas.MaxRPM) / 60.0f * 1000.0f / 330.0f;
        if (vlTarget > limit) {
            vlTarget = limit;
        }
        else if (vlTarget < -limit) {
            vlTarget = -limit;
        }
        auto vNow = ToFloat(_change) / ToFloat(_time);
        vTarget = vlTarget - vNow;
        vintegral += vTarget;
        if (vintegral > VIntegralLimit) {
            vintegral = VIntegralLimit;
        }
        else if (vintegral < -VIntegralLimit) {
            vintegral = -VIntegralLimit;
        }
        auto temp = vpara.Kp * ToFloat(vTarget) + vpara.Ki * ToFloat(vintegral) - vpara.Kd * ToFloat((vTarget - vlastError) / _time);
        vlastError = vTarget;
        return temp ;
    }


} // Motor


// Modules
//
// Created by Administrator on 24-10-12.
//

#ifndef LV_Reserve_PID_HPP
#define LV_Reserve_PID_HPP

#include "MotorU.hpp"

namespace Modules::Motor {

        enum PidType {
            Single,
            Double,
            Triple
        };

        struct PidPara {
            float Kp;
            float Ki;
            float Kd;
        };

        template<PidType>
        class Pid;

        template<>
        struct Pid<Single> {
            explicit Pid(MotorParas & _motorParas, PidPara _para,float _dieZone)
            : motorParas(_motorParas),dieZone(_dieZone) {
                cpara = _para;
            };
            using i16 = int16_t;

            PidPara cpara;
            i16 IntegralLimit = 250 * 60 / 1000;
            MotorParas & motorParas;
            i16 Target = 0;
            float Update(i16 _change,uint32_t _time);
            void Reset();;
            const float dieZone;
        private:
            i16 lastError{};
            i16 integral{};
        };


        template<>
        struct Pid<Double> {
            explicit Pid(MotorParas & _motorParas, PidPara _cpara,PidPara _vpara,float _dieZone)
            : motorParas(_motorParas),dieZone(_dieZone) {
                cpara = _cpara;
                vpara = _vpara;
            };
            PidPara cpara;
            PidPara vpara;
            int IntegralLimit = 250 * 60 / 1000;
            float VIntegralLimit = 250 * 60 / 1000;
            MotorParas & motorParas;
            int Target = 0;
            const float dieZone;
            float Update(int16_t _change,uint32_t _time);
            void Reset();
        private:
            float vTarget {};
            int clastError {};
            int cintegral {};
            float vlastError {};
            float vintegral {};
        };

        template<>
        struct Pid<Triple> {
            explicit Pid(MotorParas & _motorParas,PidPara _cPara,PidPara _tPara,PidPara _sPara,float _dieZone) :
            mParas(_motorParas),dieZone(_dieZone) {
                cPara = _cPara;
                tPara = _tPara;
                sPara = _sPara;
            };

            MotorParas mParas;

            PidPara cPara;
            PidPara tPara;
            PidPara sPara;

            const float dieZone;
        private:

            float tError = 0;
            float tLastError = 0;
            float tIntegral = 0;

            uint16_t cError = 0;
            uint16_t cLastError = 0;
            uint16_t cIntegral = 0;

            float sError = 0;
            float sLastError = 0;
            float sIntegral = 0;

            float sIntegralLimit = 100.0f;
            uint16_t cIntegralLimit = 100;
            float tIntegralLimit = 100.0f;

            void Reset();

            float Update(uint16_t _change,float _dutyCycle,uint32_t _time);

        };

} // Motor
// Modules

#endif //LV_Reserve_PID_HPP

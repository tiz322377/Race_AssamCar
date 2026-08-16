//
// Created by Administrator on 24-10-18.
//

#ifndef LV_Reserve_PLUSEIO_HPP
#define LV_Reserve_PLUSEIO_HPP

#include "Modules/StepMotor/StepMotorU.hpp"

namespace Modules {

    template<>
    class StepMotor<PlusesIO> {
        using SPin = Peripheral::GPIOPin<Peripheral::Output>;
        SPin enable;
        SPin direction;
        SPin step;

    public:

        explicit StepMotor(SPin _en, SPin _dir, SPin _stp) : enable(_en), direction(_dir), step(_stp) {
            enable.Write(false);
            direction.Write(false);
            step.Write(false);
        }

        void StepOnce(uint32_t _delay) const;

        void Steps(uint32_t _steps, uint32_t _delay) const;

        void SetDirection(bool _dir) {
            direction.Write(_dir);
        }

        void SetEnable(bool _en) {
            enable.Write(_en);
        }

    };
}

#endif //LV_Reserve_PLUSEIO_HPP

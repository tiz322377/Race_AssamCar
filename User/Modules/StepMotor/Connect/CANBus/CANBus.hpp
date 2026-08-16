//
// Created by Administrator on 25-2-28.
//

#ifndef CANBUS_HPP
#define CANBUS_HPP

#include "Modules/StepMotor/StepMotorU.hpp"
#include "Modules/StepMotor/Connect/Command.hpp"
#include "Peripheral/Can.hpp"
#include "Peripheral/FDCan.hpp"

#ifdef HAL_FDCAN_MODULE_ENABLED

namespace Modules {
    template<>
    class StepMotor<CANBus> {

        const Peripheral::FDCan<Peripheral::Classic> * fdcan;

        StepMotorCH handle;

        using Command = StepMotorCH::Command;

    public:

        explicit StepMotor(const Peripheral::FDCan<Peripheral::CANType::Classic> * _fdcan, const uint8_t _addr) : fdcan(_fdcan), handle(_addr) {}

        void ResetZeroPoint() const {
            const auto [data, size] = handle.ResetZeroPoint();
            fdcan->Transmit(data[0],data+1,size-1);
        }

        void SetEnable(const bool _enable) const {
            const auto [data, size] = handle.SetEnable(_enable);
            fdcan->Transmit(data[0],data+1,size-1);
        }

        void SetVelocity(const bool _dir, const bool _corp, const uint8_t _acc , const uint16_t _velocity) const {
            const auto [data, size] = handle.SetVelocity(_dir,_corp,_acc,_velocity);
            fdcan->Transmit(data[0],data+1,size-1);
        }

        void SetPosition(const bool _dir, const bool _corp, const uint8_t _acc, const uint16_t _velocity, const uint32_t _distance, const bool _relative) const {
            const auto [data, size] = handle.SetPosition(_dir,_corp,_acc,_velocity,_distance,_relative);
            fdcan->Transmit(data[0],data+1,size-1);
        }

        void SetSubDivision(const uint8_t _sub) const {
            const auto [data, size] = handle.SetSubDivision(_sub);
            fdcan->Transmit(data[0],data+1,size-1);
        }

        void StopNow() const {
            const auto [data, size] = handle.StopNow();
            fdcan->Transmit(data[0],data+1,size-1);
        }

        void RunSyncTask() const {
            const auto [data, size] = handle.RunSyncTask();
            fdcan->Transmit(data[0],data+1,size-1);
        }

        void Calibration() const {
            const auto [data,size] = handle.Calibration();
            fdcan->Transmit(data[0],data+1,size-1);
        }

        void ClearAngle() const {
            const auto [data,size] = handle.ClearAngle();
            fdcan->Transmit(data[0],data+1,size-1);
        }

#if 0
        bool WaitUntilStop(const uint16_t _delay = 100) const {
            uint8_t rec[4] {};
            uart->Receive(rec,4,_delay);
            if (rec[0] == handle.GetAddress()) {
                if (rec[1] == 0x00 && rec[2] == 0xEE && rec[3] == 0x6B) {
                    return false;
                }
                return true;
            }
            return false;
        }

#endif


    };
}
#endif
#endif //CANBUS_HPP

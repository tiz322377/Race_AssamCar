//
// Created by Administrator on 25-2-28.
//

#ifndef SERIAL_HPP
#define SERIAL_HPP

#include "Modules/StepMotor/StepMotorU.hpp"
#include "Modules/StepMotor/Connect/Command.hpp"
#include "Peripheral/uart.hpp"

#if CMSIS_RTOS
#include "cmsis_os2.h"
#endif

inline void delay(const uint32_t _delay) {
#if CMSIS_RTOS
    osDelay(_delay);
#else
    HAL_Delay(_delay);
#endif
}

template<>
class Modules::StepMotor<Modules::Serial> {
    const Peripheral::Uart<Peripheral::Normal> *uart;

    StepMotorCH handle;

    using Command = StepMotorCH::Command;

    using thisClass = StepMotor;

    static void delay() { ::delay(50); }

public:
    explicit StepMotor(const Peripheral::Uart<Peripheral::Normal> *_uart, const uint8_t _addr) : uart(_uart),
        handle(_addr) {
    }


    void ResetZeroPoint() const {
        const auto [data, size] = handle.ResetZeroPoint();
        uart->Send(data, size);
        delay();
    }

    void SetEnable(const bool _enable) const {
        const auto [data, size] = handle.SetEnable(_enable);
        uart->Send(data, size);
        delay();
    }

    void SetVelocity(const bool _dir, const bool _corp, const uint8_t _acc, const uint16_t _velocity) const {
        const auto [data, size] = handle.SetVelocity(_dir, _corp, _acc, _velocity);
        uart->Send(data, size);
        delay();
    }

    /**
   * @brief  Set The Motor Position
   * @param  _dir Motor Direction
   * @param  _corp Is Motor Corporate
    * @param  _acc Acceleration
    * @param  _velocity Velocity
    * @param  _distance Distance
    * @param  _relative Is Relative
   * @retval bool Success or Not
   */
    void SetPosition(const bool _dir, const bool _corp, const uint8_t _acc, const uint16_t _velocity,
                     const uint32_t _distance, const bool _relative) const {
        const auto [data, size] = handle.SetPosition(_dir, _corp, _acc, _velocity, _distance, _relative);
        uart->Send(data, size);
        delay();
    }

    void SetSegmentation(const uint8_t _sub) const {
        const auto [data, size] = handle.SetSubDivision(_sub);
        uart->Send(data, size);
        delay();
    }

    void StopNow() const {
        const auto [data, size] = handle.StopNow();
        uart->Send(data, size);
        delay();
    }


    static void RunSyncTask(const uint8_t _addr, const Peripheral::Uart<Peripheral::Normal> *_bus) {
        auto [data,size] = StepMotorCH::RunSyncTask(_addr);
        _bus->Send(data, size);
        delay();
    }

    void RunSyncTask(const uint8_t _addr) const {
        auto [data,size] = StepMotorCH::RunSyncTask(_addr);
        uart->Send(data, size);
        delay();
    }

    void Calibration() const {
        const auto [data,size] = handle.Calibration();
        uart->Send(data, size);
        delay();
    }

    void ClearAngle() const {
        const auto [data,size] = handle.ClearAngle();
        uart->Send(data, size);
        delay();
    }

    [[nodiscard]] bool WaitUntilStop(const uint16_t _delay) const {
        uint8_t rec[4]{};
        uart->Receive(rec, 4, _delay);
        if (rec[0] == handle.GetAddress()) {
            if (rec[1] == 0x00 && rec[2] == 0xEE && rec[3] == 0x6B) {
                return false;
            }
            return true;
        }
        return false;
    }

    [[nodiscard]] static bool WaitUntilStop(const Peripheral::Uart<Peripheral::Normal> *_uart, const uint16_t _delay) {
        uint8_t rec[4]{};
        _uart->Receive(rec, 4, _delay);
        if (rec[1] == 0xFF && rec[2] == 0xEE && rec[3] == 0x6B) {
            return false;
        }
        return true;
    }

    [[nodiscard]] uint8_t GetAddress() const {
        return handle.GetAddress();
    }
};

#endif //SERIAL_HPP

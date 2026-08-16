//
// Created by Administrator on 25-2-28.
//

#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <cstdint>

namespace Modules {

    //CH means Command Handle
    class StepMotorCH {
        const uint8_t addr;

    public:
        struct Command {
            uint8_t data[16] = {};
            uint8_t size = 0;

            uint8_t &operator[](const uint8_t _index) {
                return data[_index];
            }
        };

        explicit StepMotorCH(const uint8_t _addr) : addr(_addr) {
        }

        Command ResetZeroPoint() const;

        Command SetEnable(bool _enable) const;

        Command SetVelocity(bool _dir, bool _corp, uint8_t _acc, uint16_t _velocity) const;

        Command SetPosition(bool _dir, bool _corp, uint8_t _acc,
                            uint16_t _velocity, uint32_t _distance, bool _relative) const;

        Command StopNow() const;

        static Command RunSyncTask(uint8_t _addr);

        Command SetSubDivision(uint8_t _sub) const;

        Command Calibration() const;

        Command ClearAngle() const;

        uint8_t GetAddress() const { return addr; }




    };
}

#endif //COMMAND_HPP

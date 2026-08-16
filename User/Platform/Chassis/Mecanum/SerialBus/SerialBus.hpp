//
// Created by Administrator on 25-3-3.
//

#ifndef SERIALBUS_HPP
#define SERIALBUS_HPP

#include "Modules/StepMotor/Connect/Serial/Serial.hpp"
#include "Platform/Chassis/MoveTasks.hpp"
#include "Platform/Chassis/Mecanum/MecanumU.hpp"
#include "Peripheral/Uart.hpp"

#include <type_traits>

namespace Platform::Chassis {
    extern MecanumChassis<SerialBus> *MCSLBPtr;

    template<>
    class MecanumChassis<SerialBus> {
    public:
        static void Create(const uint8_t *_addrs, const Peripheral::Uart<Peripheral::Normal> *_bus, const uint8_t _cent,
                           const bool *_dirs, const bool _is18Angle, const double _radius) {
            static MecanumChassis instance(_bus, _addrs, _dirs, _cent, _is18Angle, _radius);
            MCSLBPtr = &instance;
        }

        static void SetRotateFixFactor(double _factor);


        bool RunTask(const MoveDirection _dir, const double _distance, const uint16_t _checkCount) const {
            moveAction(_dir, _distance);
            return WaitForStop(_checkCount);
        }

        [[nodiscard]] bool WaitForStop(uint16_t _checkCount) const;

    private:
        const Modules::StepMotor<Modules::Serial> lfMotor;
        const Modules::StepMotor<Modules::Serial> rfMotor;
        const Modules::StepMotor<Modules::Serial> lbMotor;
        const Modules::StepMotor<Modules::Serial> rbMotor;

        const Peripheral::Uart<Peripheral::Normal> *bus;

        bool is18Angle;

        double radius;

        struct MoveInfos {
            uint8_t acceleration;
            uint16_t velocity;
            uint32_t distance;
        };

        explicit MecanumChassis(const Peripheral::Uart<Peripheral::Normal> *_bus, const uint8_t *_addrs,
                                const bool *_dirs, const uint8_t _cent, const bool _is18Angle,
                                const double _radius): lfMotor(_bus, _addrs[0]), rfMotor(_bus, _addrs[1]),
                                                       lbMotor(_bus, _addrs[2]), rbMotor(_bus, _addrs[3]),
                                                       bus(_bus), is18Angle(_is18Angle), radius(_radius) {
            setDirection(_dirs);
            setCent(_cent);
            targetPluses = 0;
        }

        static void setDirection(const bool *_dirs);

        void enableAll() const;

        void moveForWard(MoveInfos _infos) const;

        void moveBackWard(MoveInfos _infos) const;

        void turnRight(MoveInfos _infos) const;

        void turnLeft(MoveInfos _infos) const;

        void transLeft(MoveInfos _infos) const;

        void transRight(MoveInfos _infos) const;

        void transRightForward(MoveInfos _infos) const;

        void transLeftForward(MoveInfos _infos) const;

        void transRightBackward(MoveInfos _infos) const;

        void transLeftBackward(MoveInfos _infos) const;

        void setCent(uint8_t _cent) const;

        void moveAction(MoveDirection _direction, double _distance) const;

        void allClear() const;

        [[nodiscard]] MoveInfos getMoveInfos(double _distance, double _factor) const;

        mutable uint32_t targetPluses;
    };
}


#endif //SERIALBUS_HPP

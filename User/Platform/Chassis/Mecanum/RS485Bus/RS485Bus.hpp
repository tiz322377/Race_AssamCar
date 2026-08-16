//
// Created by Administrator on 25-3-3.
//

#ifndef RS485BUS_HPP
#define RS485BUS_HPP

#include "Config/Config.hpp"
#include "Modules/StepMotor/Connect/Serial/Serial.hpp"
#include "Platform/Chassis/MoveTasks.hpp"
#include "Platform/Chassis/Mecanum/MecanumU.hpp"
#include "Modules/RS485/rs485.hpp"

#include <type_traits>

namespace Platform::Chassis {
    extern MecanumChassis<RS485Bus> * MCRSBPtr;

    template<>
    class MecanumChassis<RS485Bus> {
    public:

        void VerifyReached();

        using CPin = Modules::RS485::CPin;

        static void Create(const uint8_t *_addrs, const Peripheral::Uart<Peripheral::DMA> *_bus,CPin _pin, const uint8_t _cent,
                           const bool *_dirs, const bool _is18Angle, const double _radius,const double _motorDsitance) {
            static MecanumChassis instance(_bus,_pin, _addrs, _dirs, _cent, _is18Angle, _radius,_motorDsitance);
            MCRSBPtr = &instance;
        }


        static void SetRotateFixFactor(double _factor);

        bool RunTask(const MoveDirection _dir, const double _distance, const uint16_t _checkCount = 0) const {
            moveAction(_dir, _distance);
            return WaitForStop(_checkCount);
        }

        void RunTaskInt(const MoveDirection _dir, const double _distance) {
            moveAction(_dir, _distance);
            isMoving = true;
            WaitForStopInt();
        }

        void RunTaskNoCheck(const MoveDirection _dir, const double _distance) const {
            moveAction(_dir, _distance);
        }

        [[nodiscard]] bool WaitForStop(uint16_t _checkCount = 0) const;

        void WaitForStopInt();

        void RunTaskTime(const MoveDirection _dir, const double _distance) {
            const auto moveInfos = moveAction(_dir, _distance);
            const auto acceleration = (double)moveInfos.acceleration / 60.0;
            const auto velocity = (double)moveInfos.velocity / 60.0;
            auto cycles = (double)moveInfos.distance / this->cent;
            if (is18Angle) {
                cycles = cycles / 200.0;
            } 
            else {
                cycles = cycles / 400.0;
            }
            auto aTime = velocity / acceleration;
            auto aDistance = acceleration * aTime * aTime;
            auto vTime = (cycles - aDistance) / velocity;
            auto totalTime = aTime + vTime;
            if (totalTime < 0) {
                totalTime = 0;
            }
            Delay(totalTime * 1000.0 + 500);
        }

        bool IsMoving() const {
            return isMoving;
        }

    private:
        const CPin flowControlPin;

        void SendEnable() const {
            flowControlPin.Write(true);
        }

        void ReceiveEnable() const  {
            flowControlPin.Write(false);
        }

        const Modules::StepMotor<Modules::Serial> lfMotor;
        const Modules::StepMotor<Modules::Serial> rfMotor;
        const Modules::StepMotor<Modules::Serial> lbMotor;
        const Modules::StepMotor<Modules::Serial> rbMotor;

        const Peripheral::Uart<Peripheral::DMA> *bus;

        bool is18Angle;

        const double radius;

        const double motorDistance;

        mutable uint8_t cent;

        struct MoveInfos {
            uint8_t acceleration;
            uint16_t velocity;
            uint32_t distance;
        };

        explicit MecanumChassis(const Peripheral::Uart<Peripheral::DMA> *_bus,CPin _pin, const uint8_t *_addrs,
                                const bool *_dirs, const uint8_t _cent, const bool _is18Angle,
                                const double _radius,const double _motorDistance) : flowControlPin(_pin), lfMotor(_bus, _addrs[0]),
                                                       rfMotor(_bus, _addrs[1]), lbMotor(_bus, _addrs[2]),rbMotor(_bus, _addrs[3]),
                                                       bus(_bus), is18Angle(_is18Angle), radius(_radius),motorDistance(_motorDistance) {
            setDirection(_dirs);
            setCent(_cent);
            isMoving = false;
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

        MoveInfos moveAction(MoveDirection _direction, double _distance) const;

        void allClear() const;

        [[nodiscard]] MoveInfos getMoveInfos(double _distance, double _factor) const;

        bool isMoving;

    };
}


#endif //RS485Bus_HPP

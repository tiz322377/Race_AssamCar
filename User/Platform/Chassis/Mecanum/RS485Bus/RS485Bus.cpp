//
// Created by Administrator on 25-3-3.
//

#include "RS485Bus.hpp"

#include "Modules/StepMotor/PluseIO/PlusesIO.hpp"
#include "cmath"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include <cstdint>

static bool stdDirections[4]     = {true, true, true, true};
static bool currentDirections[4] = {true, true, true, true};
static double rotateFixFactor    = 1.41421356237;

static bool &stdLF = stdDirections[0];
static bool &stdRF = stdDirections[1];
static bool &stdLB = stdDirections[2];
static bool &stdRB = stdDirections[3];
static bool &CLF   = currentDirections[0];
static bool &CRF   = currentDirections[1];
static bool &CLB   = currentDirections[2];
static bool &CRB   = currentDirections[3];

static uint8_t verifyBuffer[32] = {};

static constexpr double pi = 3.14159265358979323846;

namespace Platform::Chassis
{
    MecanumChassis<RS485Bus> *MCRSBPtr = nullptr;

    void MecanumChassis<RS485Bus>::SetRotateFixFactor(const double _factor)
    {
        rotateFixFactor = _factor;
    }

    void MecanumChassis<RS485Bus>::setDirection(const bool *_dirs)
    {
        for (int i = 0; i < 4; i++) {
            stdDirections[i] = _dirs[i];
        }
    }

    void MecanumChassis<RS485Bus>::enableAll() const
    {
        lfMotor.SetEnable(true);
        rfMotor.SetEnable(true);
        lbMotor.SetEnable(true);
        rbMotor.SetEnable(true);
    }

    void MecanumChassis<RS485Bus>::moveForWard(const MoveInfos _infos) const
    {
        CLF = stdLF;
        CRF = stdRF;
        CLB = stdLB;
        CRB = stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::moveBackWard(const MoveInfos _infos) const
    {
        CLF = !stdLF;
        CRF = !stdRF;
        CLB = !stdLB;
        CRB = !stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::turnRight(const MoveInfos _infos) const
    {
        CLF = stdLF;
        CRF = !stdRF;
        CLB = stdLB;
        CRB = !stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::turnLeft(const MoveInfos _infos) const
    {
        CLF = !stdLF;
        CRF = stdRF;
        CLB = !stdLB;
        CRB = stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::transLeft(const MoveInfos _infos) const
    {
        CLF = !stdLF;
        CLB = stdLB;
        CRF = stdRF;
        CRB = !stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::transRight(const MoveInfos _infos) const
    {

        CLF = stdLF;
        CLB = !stdLB;
        CRF = !stdRF;
        CRB = stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::transRightForward(const MoveInfos _infos) const
    {
        CLF = stdLF;
        CRB = !stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::transLeftForward(const MoveInfos _infos) const
    {
        CLB = !stdLB;
        CRF = stdRF;

        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::transRightBackward(const MoveInfos _infos) const
    {
        CLB = stdLB;
        CRF = !stdRF;

        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::transLeftBackward(const MoveInfos _infos) const
    {
        CLF = !stdLF;
        CRB = stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
    }

    void MecanumChassis<RS485Bus>::setCent(const uint8_t _cent) const
    {
        cent = _cent;
        lfMotor.SetSegmentation(_cent);
        rfMotor.SetSegmentation(_cent);
        lbMotor.SetSegmentation(_cent);
        rbMotor.SetSegmentation(_cent);
    }

    using thisClass = MecanumChassis<RS485Bus>;

    thisClass::MoveInfos MecanumChassis<RS485Bus>::moveAction(const MoveDirection _direction, const double _distance) const
    {
        SendEnable();
        allClear();
        if (_direction == Rotate) {
            double rotateDistance = std::abs(_distance);
            rotateDistance        = rotateDistance / 360.0 * 2 * pi * motorDistance;
            const auto ms         = getMoveInfos(rotateDistance, rotateFixFactor);
            if (_distance < 0) {
                turnLeft(ms);
            } else {
                turnRight(ms);
            }
            Modules::StepMotor<Modules::Serial>::RunSyncTask(0, bus);
            return ms;
        }
        const auto moveInfos = getMoveInfos(_distance, 1.0);
        switch (_direction) {
            case Forward:
                moveForWard(moveInfos);
                break;
            case Backward:
                moveBackWard(moveInfos);
                break;
            case Right:
                transRight(moveInfos);
                break;
            case Left:
                transLeft(moveInfos);
                break;
            case RightForward:
                transRightForward(moveInfos);
                break;
            case LeftForward:
                transLeftForward(moveInfos);
                break;
            case RightBackward:
                transRightBackward(moveInfos);
                break;
            case LeftBackward:
                transLeftBackward(moveInfos);
                break;
            default:
                break;
        }
        Modules::StepMotor<Modules::Serial>::RunSyncTask(0, bus);
        return moveInfos;
    }

    void MecanumChassis<RS485Bus>::allClear() const
    {
        lfMotor.ClearAngle();
        rfMotor.ClearAngle();
        lbMotor.ClearAngle();
        rbMotor.ClearAngle();
    }

    thisClass::MoveInfos thisClass::getMoveInfos(const double _distance, const double _factor) const
    {
        const double centTemp    = cent;
        const double circles     = _distance / radius / 2 / pi;
        uint32_t pluses          = 0;
        uint32_t plusesPerCircle = 0;
        if (is18Angle) {
            plusesPerCircle = static_cast<uint32_t>(200.0 * centTemp * _factor);
        } else {
            plusesPerCircle = static_cast<uint32_t>(400.0 * centTemp * _factor);
        }
        pluses = static_cast<uint32_t>(static_cast<double>(plusesPerCircle) * circles);
        MoveInfos infos{};
        infos.acceleration = pluses / 10 * 60 / plusesPerCircle;
        if (infos.acceleration <= 5) {
            infos.acceleration = 5;
        }
        if (infos.acceleration > 20) {
            infos.acceleration = 20;
        }
        infos.velocity = pluses / 5 * 60 / plusesPerCircle;
        if (infos.velocity <= 10) {
            infos.velocity = 10;
        }
        if (infos.velocity > 60) {
            infos.velocity = 60;
        }
        infos.distance = pluses;
        return infos;
    }

    bool MecanumChassis<RS485Bus>::WaitForStop(uint16_t _checkCount) const
    {
        ReceiveEnable();
        uint16_t times = 0;
        for (;;) {
            //@Command Read
            uint8_t data[3]{0, 0x3A, 0x6b};
            constexpr uint8_t resultLength = 4;
            uint8_t receive[4][resultLength]{};
            bool result[4]{true, true, true, true};

            data[0] = lfMotor.GetAddress();
            SendEnable();
            bus->AbortReceiveIT();
            bus->Send(data, 3);
            ReceiveEnable();
            bus->Receive(receive[0], resultLength, 100);
            result[0] &= receive[0][0] == lfMotor.GetAddress();
            result[0] &= receive[0][1] == 0X3A;
            result[0] &= (receive[0][2] & 0x02) == 0X02;
            result[0] &= receive[0][3] == 0X6b;

            delay(5);

            data[0] = rfMotor.GetAddress();
            SendEnable();
            bus->AbortReceiveIT();
            bus->Send(data, 3);
            ReceiveEnable();
            bus->Receive(receive[1], resultLength, 100);
            result[1] &= receive[1][0] == rfMotor.GetAddress();
            result[1] &= receive[1][1] == 0X3A;
            result[1] &= (receive[1][2] & 0x02) == 0X02;
            result[1] &= receive[1][3] == 0X6b;

            delay(5);

            data[0] = lbMotor.GetAddress();
            SendEnable();
            bus->AbortReceiveIT();
            bus->Send(data, 3);
            ReceiveEnable();
            bus->Receive(receive[2], resultLength, 100);
            result[2] &= receive[2][0] == lbMotor.GetAddress();
            result[2] &= receive[2][1] == 0X3A;
            result[2] &= (receive[2][2] & 0x02) == 0X02;
            result[2] &= receive[2][3] == 0X6b;

            delay(5);

            data[0] = rbMotor.GetAddress();
            SendEnable();
            bus->AbortReceiveIT();
            bus->Send(data, 3);
            ReceiveEnable();
            bus->Receive(receive[3], resultLength, 100);
            result[3] &= receive[3][0] == rbMotor.GetAddress();
            result[3] &= receive[3][1] == 0X3A;
            result[3] &= (receive[3][2] & 0x02) == 0X02;
            result[3] &= receive[3][3] == 0X6b;

            uint8_t count = 0;

            for (const auto i : result) {
                if (i) {
                    count++;
                }
            }

            if (count >= 1) {
                return true;
            }

            times++;

            if (_checkCount != 0 && times >= _checkCount) {
                return false;
            }
        }
    }

    void MecanumChassis<RS485Bus>::VerifyReached()
    {
        bool result = true;
        result &= verifyBuffer[1] == 0xFD;
        result &= verifyBuffer[2] == 0x9F;
        result &= verifyBuffer[3] == 0x6B;
        isMoving = !result;
    }

    void MecanumChassis<RS485Bus>::WaitForStopInt()
    {
        for (auto &i : verifyBuffer) {
            i = 0;
        }
        ReceiveEnable();
        bus->ReceiveDMA(verifyBuffer, 1024);
        while (!(verifyBuffer[1] == 0xFD &&
               verifyBuffer[2] == 0x9F &&
               verifyBuffer[3] == 0x6B));
        isMoving = false;
    }
}

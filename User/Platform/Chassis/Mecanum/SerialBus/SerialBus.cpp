//
// Created by Administrator on 25-3-3.
//

#include "SerialBus.hpp"

#include "Modules/StepMotor/PluseIO/PlusesIO.hpp"

static uint8_t cent = 16;

static bool stdDirections[4] = {true, true, true, true};
static bool currentDirections[4] = {true, true, true, true};
static double rotateFixFactor = 1.05;

static bool& stdLF =  stdDirections[0];
static bool& stdRF =  stdDirections[1];
static bool& stdLB =  stdDirections[2];
static bool& stdRB =  stdDirections[3];
static bool& CLF = currentDirections[0];
static bool& CRF = currentDirections[1];
static bool& CLB = currentDirections[2];
static bool& CRB = currentDirections[3];

static constexpr double pi = 3.14159265358979323846;

namespace Platform::Chassis {
    MecanumChassis<SerialBus> * MCSLBPtr = nullptr;

    void MecanumChassis<SerialBus>::SetRotateFixFactor(const double _factor) {
        rotateFixFactor = _factor;
    }

    void MecanumChassis<SerialBus>::setDirection(const bool *_dirs) {
        for (int i = 0; i < 4; i++) {
            stdDirections[i] = _dirs[i];
        }
    }

    void MecanumChassis<SerialBus>::enableAll() const {
        lfMotor.SetEnable(true);
        rfMotor.SetEnable(true);
        lbMotor.SetEnable(true);
        rbMotor.SetEnable(true);
    }

    void MecanumChassis<SerialBus>::moveForWard(const MoveInfos _infos) const {
        CLF = stdLF;
        CRF = stdRF;
        CLB = stdLB;
        CRB = stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }

    void MecanumChassis<SerialBus>::moveBackWard(const MoveInfos _infos) const {
        CLF = !stdLF;
        CRF = !stdRF;
        CLB = !stdLB;
        CRB = !stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }

    void MecanumChassis<SerialBus>::turnRight(const MoveInfos _infos) const {
        CLF = !stdLF;
        CRF = stdRF;
        CLB = !stdLB;
        CRB = stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }

    void MecanumChassis<SerialBus>::turnLeft(const MoveInfos _infos) const {
        CLF = stdLF;
        CRF = !stdRF;
        CLB = stdLB;
        CRB = !stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }

    void MecanumChassis<SerialBus>::transLeft(const MoveInfos _infos) const {
        CLF = stdLF;
        CLB = !stdLB;
        CRF = !stdRF;
        CRB = stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }

    void MecanumChassis<SerialBus>::transRight(const MoveInfos _infos) const {
        CLF = !stdLF;
        CLB = stdLB;
        CRF = stdRF;
        CRB = !stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }

    void MecanumChassis<SerialBus>::transRightForward(const MoveInfos _infos) const {
        CLF = stdLF;
        CRB = !stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }

    void MecanumChassis<SerialBus>::transLeftForward(const MoveInfos _infos) const {
        CLB = !stdLB;
        CRF = stdRF;

        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }

    void MecanumChassis<SerialBus>::transRightBackward(const MoveInfos _infos) const {
        CLB = stdLB;
        CRF = !stdRF;

        lbMotor.SetPosition(CLB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rbMotor.SetPosition(CRB, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }

    void MecanumChassis<SerialBus>::transLeftBackward(const MoveInfos _infos) const {
        CLF = !stdLF;
        CRB = stdRB;

        lfMotor.SetPosition(CLF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);
        rfMotor.SetPosition(CRF, true, _infos.acceleration, _infos.velocity, _infos.distance, true);

        targetPluses = _infos.distance;
    }


    void MecanumChassis<SerialBus>::setCent(const uint8_t _cent) const {
        cent = _cent;
        lfMotor.SetSegmentation(_cent);
        rfMotor.SetSegmentation(_cent);
        lbMotor.SetSegmentation(_cent);
        rbMotor.SetSegmentation(_cent);
    }

    void MecanumChassis<SerialBus>::moveAction(const MoveDirection _direction, const double _distance) const {
        allClear();

        if (_direction == Rotate) {
            double rotateDistance = 0.0;
            if (_distance < 0) {
                rotateDistance = _distance;
            }
            rotateDistance = rotateDistance / 360.0 * radius;
            const auto ms = getMoveInfos(rotateDistance, rotateFixFactor);
            if (_distance < 0) {
                turnLeft(ms);
            } else {
                turnRight(ms);
            }
            Modules::StepMotor<Modules::Serial>::RunSyncTask(0, bus);
            return;
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
    }

    void MecanumChassis<SerialBus>::allClear() const {
        lfMotor.ClearAngle();
        rfMotor.ClearAngle();
        lbMotor.ClearAngle();
        rbMotor.ClearAngle();
    }

    using thisClass = MecanumChassis<SerialBus>;

    thisClass::MoveInfos thisClass::getMoveInfos(const double _distance, const double _factor) const {
        const double centTemp = cent;
        const double circles = _distance / radius / 2 / pi;
        uint32_t pluses = 0;
        if (is18Angle) {
            pluses = static_cast<uint32_t>(circles * 200.0 * centTemp * _factor);
        }
        else {
            pluses = static_cast<uint32_t>(circles * 400.0 * centTemp * _factor);
        }

        MoveInfos infos{};
        const uint16_t circleCount = circles;

        infos.acceleration = pluses / 20;
        if (infos.acceleration == 0) {
            infos.acceleration = 1;
        }
        infos.velocity = pluses / 5;
        if (infos.velocity == 0) {
            infos.velocity = 1;
        }
        infos.distance = pluses;

        return infos;
    }

    bool MecanumChassis<SerialBus>::WaitForStop(const uint16_t _checkCount) const {
        uint16_t times = 0;
        for (;;) {
            //@Command Read
            uint8_t data[4]{0, 0x3A, 0x6b};
            constexpr uint8_t resultLength = 4;
            uint8_t receive[4][resultLength]{};
            bool result[4] {true,true,true,true};

            uint8_t buffer[256];
            bus->Receive(buffer, sizeof(buffer),100);

            delay(100);
            data[0] = lfMotor.GetAddress();
            bus->Send(data, 3);
            bus->Receive(receive[0], resultLength, 100);
            result[0] &= receive[0][0] == lfMotor.GetAddress();
            result[0] &= receive[0][1] == 0X3A;
            result[0] &= (receive[0][2]&0x02) == 0X02;
            result[0] &= receive[0][3] == 0X6b;

            delay(100);
            data[0] = rfMotor.GetAddress();
            bus->Send(data, 3);
            bus->Receive(receive[1], resultLength, 100);
            result[1] &= receive[1][0] == rfMotor.GetAddress();
            result[1] &= receive[1][1] == 0X3A;
            result[1] &= (receive[1][2]&0x02) == 0X02;
            result[1] &= receive[1][3] == 0X6b;

            delay(100);
            data[0] = lbMotor.GetAddress();
            bus->Send(data, 3);
            bus->Receive(receive[2], resultLength, 100);
            result[2] &= receive[2][0] == lbMotor.GetAddress();
            result[2] &= receive[2][1] == 0X3A;
            result[2] &= (receive[2][2]&0x02) == 0X02;
            result[2] &= receive[2][3] == 0X6b;

            delay(100);
            data[0] = rbMotor.GetAddress();
            bus->Send(data, 3);
            bus->Receive(receive[3], resultLength, 100);
            result[3] &= receive[3][0] == rbMotor.GetAddress();
            result[3] &= receive[3][1] == 0X3A;
            result[3] &= (receive[3][2]&0x02) == 0X02;
            result[3] &= receive[3][3] == 0X6b;

            uint8_t count = 0;

            for (const auto i : result) {
                if (result[i]) {
                    count++;
                }
            }

            if (count >= 2) {
                return true;
            }

            if (times == 0) {
                continue;
            }

            times++;

            if (times >= _checkCount) {
                return false;
            }

        }
    }
}

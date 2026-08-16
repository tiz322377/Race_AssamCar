//
// Created by Administrator on 24-10-12.
//

#include "StepBus.hpp"
#include "Peripheral/GPIO.hpp"
#include "usart.h"
#include <cstdint>
#include <cstring>

#include "Config/Config.hpp"

#include "Peripheral/Delay.hpp"

static double minDistance = 0.55;

static constexpr double pi = 3.141592653589793238462643;

using CPin = Peripheral::GPIOPin<Peripheral::Output>;

struct delayPack{
    int ms;
    int us;
};

#ifdef S_DEBUG
#include "Screen.hpp"
#endif

namespace Platform::Chassis {

    enum Xcent {
        cent = 2,
    };

    MecanumChassis<StepBus> * MCSBPtr = nullptr;

    MecanumChassis<StepBus>::stepMotor::stepMotor(CPin _en, CPin _dir) : Enable(_en), Direction(_dir) {
        Direction.Write(true);
    }

    void MecanumChassis<StepBus>::init() {
        targetSteps = 0;
        waitCount = 0;
        tasks.SetMoveFunc([](MoveDirection _direction, float _distance) {
            MCSBPtr->moveAction(_direction, _distance);
        });
        state = Lock;
        minDistance = MCSBPtr->radius / 20.0;
    }

//#define float(x) static_cast<float>(x)
//#define double(x) static_cast<double>(x)

#define To(x) static_cast<x>

    void MecanumChassis<StepBus>::moveAction(const MoveDirection _direction,const double _distance) {
        static constexpr auto s2 = 1.4142135623730950488016887242097 * 1.075 * 0.99111 / 0.994444444;
        if (_direction == Rotate) {
            if (_distance > 0) {
                turnLeft();
            }
            else {
                turnRight();
            }
            auto angle = _distance / 180.0 * pi;
            if (angle > 0)
                setSteps(static_cast<int32_t>((angle * motorDistance / minDistance / s2) * (To(double)(cent))));
            else
                setSteps(static_cast<int32_t>(-(angle * motorDistance / minDistance / s2) * (To(double)(cent))));
            state = _direction;
        }
        else {
            switch (_direction) {
                case Forward:
                    moveForWard();
                    break;
                case Backward:
                    moveBackWard();
                    break;
                case Left:
                    transLeft();
                    break;
                case Right:
                    transRight();
                    break;
                case LeftForward:
                    transLeftForward();
                    break;
                case RightForward:
                    transRightForward();
                    break;
                case LeftBackward:
                    transLeftBackward();
                    break;
                case RightBackward:
                    transRightBackward();
                    break;
                case Lock:
                case Rotate:
                    break;
            }
            setSteps(static_cast<int32_t>(_distance / minDistance * static_cast<double>(cent / 2)));
            state = _direction;
        }
#ifdef S_DEBUG
    //Screen::UpdateInf(_direction, targetSteps);
#endif
    }

    void MecanumChassis<StepBus>::enableAll() const {
        lf.Enable.Write(true);
        rf.Enable.Write(true);
        lb.Enable.Write(true);
        rb.Enable.Write(true);
    }

    void MecanumChassis<StepBus>::moveForWard() const {
        lf.Direction.Write(true);
        rf.Direction.Write(true);
        lb.Direction.Write(true);
        rb.Direction.Write(true);

        enableAll();
    }

    void MecanumChassis<StepBus>::moveBackWard() const {
        lf.Direction.Write(false);
        rf.Direction.Write(false);
        lb.Direction.Write(false);
        rb.Direction.Write(false);

        enableAll();
    }

    void MecanumChassis<StepBus>::turnRight() const {
        lf.Direction.Write(true);
        rf.Direction.Write(false);
        lb.Direction.Write(true);
        rb.Direction.Write(false);

        enableAll();
    }

    void MecanumChassis<StepBus>::turnLeft() const {
        lf.Direction.Write(false);
        lb.Direction.Write(false);

        rf.Direction.Write(true);
        rb.Direction.Write(true);

        enableAll();

    }

    void MecanumChassis<StepBus>::transLeft() const {
        lf.Direction.Write(false);
        lb.Direction.Write(true);

        rf.Direction.Write(true);
        rb.Direction.Write(false);

        enableAll();

    }

    void MecanumChassis<StepBus>::transRight() const {

        lf.Direction.Write(true);
        lb.Direction.Write(false);

        rf.Direction.Write(false);
        rb.Direction.Write(true);

        enableAll();
    }

    void MecanumChassis<StepBus>::transLeftForward() const  {
        rf.Direction.Write(true);
        lb.Direction.Write(true);

        rb.Enable.Write(false);
        lf.Enable.Write(false);

    }

    void MecanumChassis<StepBus>::transRightForward() const {
        lf.Direction.Write(true);
        rb.Direction.Write(true);

        rf.Enable.Write(false);
        lb.Enable.Write(false);

    }

    void MecanumChassis<StepBus>::transRightBackward() const {
        rf.Direction.Write(false);
        lb.Direction.Write(false);

        rb.Enable.Write(false);
        lf.Enable.Write(false);

    }

    void MecanumChassis<StepBus>::transLeftBackward() const {
        rf.Direction.Write(false);
        lb.Direction.Write(false);

        rb.Enable.Write(false);
        lf.Enable.Write(false);
    }
#ifdef CMSIS_RTOS
    void MecanumChassis<StepBus>::AddTask(const MoveDirection _direction, const float _distance) {
        tasks.AddTasks({_direction,_distance});
    }

    void MecanumChassis<StepBus>::RunTask() {
        if (tasks.IsEmpty()) {
            return;
        }
        if (IsMoveComplete())
            tasks.RunTask();
        else
            waitCount += 1;
    }

    void MecanumChassis<StepBus>::MoveCompleteCallBack() {
        targetSteps = 0;
        state = Lock;
        if (waitCount > 0) {
            waitCount -= 1;
            RunTask();
        }
    }
#else
    void MecanumChassis<StepBus>::RunTask(Platform::Chassis::MoveDirection _direction, double _distance) {
        moveAction(_direction, _distance);
        HAL_Delay(50);
        while (!IsMoveComplete()) {
            Update();
        }
        HAL_Delay(750);
    }

#endif

    bool MecanumChassis<StepBus>::IsMoveComplete() const {
        return targetSteps == 0;
    }

    MoveDirection MecanumChassis<StepBus>::GetState() const {
        return state;
    }

    void MecanumChassis<StepBus>::reset() {
        setSteps(0);
        state = Lock;
    }

    void MecanumChassis<StepBus>::Update() {
        if (targetSteps < 5) {
            nanoUpdate();
        }
        if (targetSteps < 200) {
            microUpdate();
            return;
        }
        if (targetSteps < 900 || state == Rotate ) {
            stableUpdate();
        }
        else {
            stableUpdate();
        }
    }

    void MecanumChassis<StepBus>::stepOnce(const double _delay) const {
        pBus.Write(true);
        Peripheral::Delay::HighResDelay(0.002);
        pBus.Write(false);
        Peripheral::Delay::HighResDelay(_delay - 0.002);
    }

    void MecanumChassis<StepBus>::curveUpdate() {
        if (targetSteps - currentSteps <= 350) {
            const auto speed = linearGenerator.interpolate((targetSteps - currentSteps));
            const auto delay = 1.0 / speed * 100.0;
            stepOnce(delay);
            currentSteps --;

            return;
        }
        if (currentSteps <= 350 && currentSteps > 0) {
            const auto speed = linearGenerator.interpolate( currentSteps);
            const auto delay = 1.0/speed * 100.0;
            stepOnce(delay);
            currentSteps --;
            return;
        }
        if (currentSteps > 0) {
            stepOnce(0.5);
            currentSteps -= 1;
        }
        else {
            reset();
        }
    }

    void MecanumChassis<StepBus>::stableUpdate() {
        if (targetSteps - currentSteps < 7) {
            stepOnce(10 - (targetSteps - currentSteps));
            currentSteps --;
            return;
        }
        if (currentSteps < 7 && currentSteps > 0) {
            stepOnce(10 - currentSteps);
            currentSteps --;
            return;
        }
        if (currentSteps > 0) {
            stepOnce(3);
            currentSteps -= 1;
        }
        else {
            reset();
        }
    }

    void MecanumChassis<StepBus>::microUpdate() {
        if (currentSteps > 0) {
            stepOnce(10);
            currentSteps -= 1;
        }
        else {
            reset();
        }
    }

    void MecanumChassis<StepBus>::nanoUpdate() {
        if (currentSteps > 0) {
            stepOnce(25);
            currentSteps -= 1;
        }
        else {
            reset();
        }
    }

    void MecanumChassis<StepBus>::WaitMoveComplete() const {
        while(!IsMoveComplete()) {
            Delay(50.0);
        }
    }

}


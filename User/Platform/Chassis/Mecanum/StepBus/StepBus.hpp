//
// Created by Administrator on 24-10-12.
//

#ifndef LV_Reserve_STEPBUS_HPP
#define LV_Reserve_STEPBUS_HPP

#include <cstdint>
#include "Platform/Chassis/MoveTasks.hpp"
#include "Platform/Chassis/Mecanum/MecanumU.hpp"
#include "Peripheral/GPIO.hpp"
#include "Peripheral/Tim.hpp"

namespace Platform::Chassis {

    extern MecanumChassis<StepBus> * MCSBPtr;

    template<>
    class MecanumChassis<StepBus> {
    public:
        using CPin = Peripheral::GPIOPin<Peripheral::Output>;
        using Para = Modules::Motor::MotorParas;
        //using PPin = Peripheral::PwmChannel;

        static void Create(
                CPin _lfEn, CPin _lfDir,
                CPin _rfEn, CPin _rfDir,
                CPin _lbEn, CPin _lbDir,
                CPin _rbEn, CPin _rbDir,
                CPin _pbus, const float _motorDistance, const float _radius
        ) {
            static MecanumChassis instance(_lfEn, _lfDir, _rfEn, _rfDir, _lbEn, _lbDir, _rbEn, _rbDir,_pbus,_motorDistance,_radius);
            MCSBPtr = &instance;
        };


#ifdef CMSIS_RTOS
        void AddTask(MoveDirection _direction, float _distance);
        void RunTask();
        void MoveCompleteCallBack();
#else
        void RunTask(MoveDirection _direction, double _distance);
#endif

        [[nodiscard]] bool IsMoveComplete() const;

        void Update();

        MoveDirection GetState() const;

        void WaitMoveComplete() const ;

    private:

        void init();

        void stepOnce(double _delay) const;

        int32_t targetSteps{};

        int32_t currentSteps{};

        unsigned short waitCount;

        const double motorDistance;

        void setSteps(const int32_t _steps) {
            targetSteps = _steps;
            currentSteps = _steps;
        };

        void moveForWard() const;

        void moveBackWard() const;

        void turnRight() const;

        void turnLeft() const;

        void transLeft() const;

        void transRight() const;

        void transRightForward() const;

        void transLeftForward() const;

        void transRightBackward() const;

        void transLeftBackward() const;

        void enableAll() const;

        void moveAction(MoveDirection _direction, double _distance);

        void reset();

        void curveUpdate();

        void stableUpdate();

        void microUpdate();

        void nanoUpdate();

        MoveDirection state;

        MoveTasks<64> tasks;

        class stepMotor {
        public:
            stepMotor(CPin _en, CPin _dir);
            CPin Enable;
            CPin Direction;
        };

        stepMotor lf;
        stepMotor rf;
        stepMotor lb;
        stepMotor rb;

        class LinearGenerator {
        private:
            double xMax, yMax, xMin, yMin;
            double slope; // 斜率
            double intercept; // 截距
        public:
            explicit LinearGenerator(double _xMax, double _yMax, double _xMin, double _yMin) {
                xMax = _xMax;
                yMax = _yMax;
                xMin = _xMin;
                yMin = _yMin;

                slope = (yMin - yMax) / (xMin - xMax);
                intercept = yMax - slope * xMax;
            }

            [[nodiscard]] double interpolate(double x) const {
                return slope * x + intercept;
            }
        };

        LinearGenerator linearGenerator;

        CPin pBus;

        float radius;

        explicit MecanumChassis(
                CPin _lfEn, CPin _lfDir,
                CPin _rfEn, CPin _rfDir,
                CPin _lbEn, CPin _lbDir,
                CPin _rbEn, CPin _rbDir,
                CPin _pbus,double _motorDistance,
                float _radius
        ) :
                lf(_lfEn, _lfDir),
                rf(_rfEn, _rfDir),
                lb(_lbEn, _lbDir),
                rb(_rbEn, _rbDir), pBus(_pbus), motorDistance(_motorDistance),
                linearGenerator(350, 200, 0, 5),radius(_radius)
        {
            init();
        }

    };


}
#endif //LV_Reserve_STEPBUS_HPP

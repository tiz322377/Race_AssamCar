#include "RobotControl.hpp"

#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace Program {
namespace {

constexpr double yawToleranceDegrees = 1.0;

double normalizeAngleDelta(double _angleDegrees)
{
    while (_angleDegrees > 180.0) {
        _angleDegrees -= 360.0;
    }
    while (_angleDegrees < -180.0) {
        _angleDegrees += 360.0;
    }
    return _angleDegrees;
}

void rotateWithYawCorrection(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const double _targetDegrees)
{
    float startYawDegrees = 0.0f;
    const bool hasStartYaw = _hardware.attitudeSensor.ReadYaw(startYawDegrees);

    _chassis.RunTaskTime(MoveDirection::Rotate, _targetDegrees);

    if (!hasStartYaw || std::abs(_targetDegrees) <= yawToleranceDegrees) {
        return;
    }

    float endYawDegrees = 0.0f;
    if (!_hardware.attitudeSensor.ReadYaw(endYawDegrees)) {
        return;
    }

    const double measuredSensorDelta = normalizeAngleDelta(
        static_cast<double>(endYawDegrees) - static_cast<double>(startYawDegrees));
    if (std::abs(measuredSensorDelta) <= yawToleranceDegrees) {
        return;
    }

    const bool commandIsClockwise = _targetDegrees > 0.0;
    const bool sensorDeltaIsPositive = measuredSensorDelta > 0.0;
    const double sensorToCommandSign = commandIsClockwise == sensorDeltaIsPositive ? 1.0 : -1.0;
    const double measuredCommandDelta = measuredSensorDelta * sensorToCommandSign;
    const double correctionDegrees = _targetDegrees - measuredCommandDelta;

    if (std::abs(correctionDegrees) > yawToleranceDegrees) {
        _chassis.RunTaskTime(MoveDirection::Rotate, correctionDegrees);
    }
}

} // namespace

RobotHardware::RobotHardware()
    : scanner(&huart1),
      hmi(&huart5),
      camera(&huart3),
      bus(&huart4),
      attitudeSensor(&hi2c1),
      gimbal(&htim1, TIM_CHANNEL_1),
      plate(&htim1, TIM_CHANNEL_2),
      arm(&htim1, TIM_CHANNEL_3),
      elevationEnable(GPIOA, GPIO_PIN_6),
      elevationPwm(&htim2, TIM_CHANNEL_1),
      elevationDirection(GPIOA, GPIO_PIN_4)
{
}

void move(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const MoveDirection _direction,
    const double _distanceMm)
{
    // if (_direction == MoveDirection::Rotate) {
    //     rotateWithYawCorrection(_hardware, _chassis, _distanceMm);
    //     return;
    // }

    _chassis.RunTaskTime(_direction, _distanceMm);
}

void printHmi(RobotHardware &_hardware, const char *_format, ...)
{
    std::array<char, 512> buffer{};

    va_list args;
    va_start(args, _format);
    std::vsnprintf(buffer.data(), buffer.size(), _format, args);
    va_end(args);

    _hardware.hmi.Send(
        reinterpret_cast<uint8_t *>(buffer.data()),
        static_cast<uint16_t>(std::strlen(buffer.data())),
        300);

    while (HAL_UART_GetState(&huart5) == HAL_UART_STATE_BUSY_TX) {
    }
}

void printHmiValue(RobotHardware &_hardware, const uint8_t _value)
{
    printHmi(
        _hardware,
        "j0.val=%u\xff\xff\xff",
        static_cast<unsigned int>(_value));
}

void printHmiText(RobotHardware &_hardware, const char *_text)
{
    printHmi(_hardware, "t1.txt=\"%s\"\xff\xff\xff", _text);
}

Rs485Chassis &init(RobotHardware &_hardware)
{
    constexpr double radiusMm = 37.5;
    const double motorDistanceMm = std::sqrt(105.0 * 105.0 + 105.0 * 105.0);
    constexpr std::array<uint8_t, 4> addresses{1, 2, 3, 4};
    constexpr std::array<bool, 4> directions{false, false, false, false};
    const OutputPin flowControlPin(GPIOF, GPIO_PIN_8);

    _hardware.gimbal.Start();
    _hardware.plate.Start();
    _hardware.arm.Start();

    // The elevation stepper driver's enable input is active-low.
    _hardware.elevationEnable.Write(false);
    _hardware.elevationPwm.SetCompare(84);

    Rs485Chassis &chassis = Rs485Chassis::Create(
        addresses.data(),
        &_hardware.bus,
        flowControlPin,
        16,
        directions.data(),
        true,
        radiusMm,
        motorDistanceMm);

    HAL_Delay(500);
    return chassis;
}

void resetMechanism(RobotHardware &_hardware)
{
    _hardware.plate.SetCompare(plateCompare[0]);
    HAL_Delay(50);
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(50);
    _hardware.arm.SetCompare(armPlaceCompare);
    HAL_Delay(50);
}

void moveElevation(
    RobotHardware &_hardware,
    const double _distanceMm,
    const ElevationDirection _direction)
{
    HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);

    // Keep the driver enabled while selecting the direction.  With EN high the
    // mechanism can fall under gravity, but the motor has no torque to move up.
    _hardware.elevationEnable.Write(false);

    if (_direction == ElevationDirection::Down) {
        _hardware.elevationDirection.Write(true);
    } else {
        _hardware.elevationDirection.Write(false);
    }

    HAL_Delay(20);

    const double steps = _distanceMm / 14.3 * 200.0;
    ElevationPulseCounterReset(static_cast<uint32_t>(steps));

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

    while (!ElevationPulseCounterIsCompleted()) {
    }
}

} // namespace Program

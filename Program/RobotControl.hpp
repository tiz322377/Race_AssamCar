#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

#include "ProgramContext.hpp"

namespace Program {

void move(
    Rs485Chassis &_chassis,
    MoveDirection _direction,
    double _distanceMm);

void printHmi(RobotHardware &_hardware, const char *_format, ...);
void printHmiValue(RobotHardware &_hardware, uint8_t _value);
void printHmiText(RobotHardware &_hardware, const char *_text);

Rs485Chassis &init(RobotHardware &_hardware);
void resetMechanism(RobotHardware &_hardware);
void moveElevation(
    RobotHardware &_hardware,
    double _distanceMm,
    ElevationDirection _direction);

} // namespace Program

#endif // ROBOT_CONTROL_HPP

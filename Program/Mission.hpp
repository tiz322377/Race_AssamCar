#ifndef MISSION_HPP
#define MISSION_HPP

#include "ProgramContext.hpp"

namespace Program {

void scanMission(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    Mission &_mission,
    QrDirection _direction);

} // namespace Program

#endif // MISSION_HPP

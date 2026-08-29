#ifndef STAGES_HPP
#define STAGES_HPP

#include "ProgramContext.hpp"

namespace Program {

void runRawStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const Mission &_mission,
    Batch _batch);

void runRoughStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const Mission &_mission,
    Batch _batch,
    uint8_t &_currentRoughSlot);

void runReplaceStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const Mission &_mission,
    Batch _batch,
    uint8_t &_currentRoughSlot);

void runBufferStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const Mission &_mission,
    Batch _batch,
    uint8_t &_currentRoughSlot);

void runHomeStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    QrDirection _direction);

} // namespace Program

#endif // STAGES_HPP

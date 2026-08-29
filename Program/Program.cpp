#include "Program.hpp"

#include "Mission.hpp"
#include "ProgramContext.hpp"
#include "RobotControl.hpp"
#include "Stages.hpp"

extern "C" [[noreturn]] void Main()
{
    Program::RobotHardware hardware;
    Program::Mission mission;
    uint8_t currentRoughSlot = Program::roughCenterSlot;
    constexpr Program::QrDirection qrDirection = Program::QrDirection::Left;

    Program::Rs485Chassis &chassis = Program::init(hardware);
    Program::resetMechanism(hardware);
    Program::scanMission(
        hardware,
        chassis,
        mission,
        qrDirection);

    Program::runRawStage(
        hardware,
        chassis,
        mission,
        Program::Batch::First);
    Program::runRoughStage(
        hardware,
        chassis,
        mission,
        Program::Batch::First,
        currentRoughSlot);
    Program::runReplaceStage(
        hardware,
        chassis,
        mission,
        Program::Batch::First,
        currentRoughSlot);
    Program::runBufferStage(
        hardware,
        chassis,
        mission,
        Program::Batch::First,
        currentRoughSlot);

    Program::runRawStage(
        hardware,
        chassis,
        mission,
        Program::Batch::Second);
    Program::runRoughStage(
        hardware,
        chassis,
        mission,
        Program::Batch::Second,
        currentRoughSlot);
    Program::runReplaceStage(
        hardware,
        chassis,
        mission,
        Program::Batch::Second,
        currentRoughSlot);
    Program::runBufferStage(
        hardware,
        chassis,
        mission,
        Program::Batch::Second,
        currentRoughSlot);

    Program::runHomeStage(hardware, chassis, qrDirection);

    for (;;) {
    }
}

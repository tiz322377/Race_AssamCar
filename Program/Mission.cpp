#include "Mission.hpp"

#include "RobotControl.hpp"

#include "main.h"
#include "usart.h"

#include <array>
#include <cstdio>

namespace Program {

void scanMission(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    Mission &_mission,
    const QrDirection _direction)
{
    std::array<char, 16> scannerMessage{};
    std::array<int, 12> missionData{};

    move(_hardware, _chassis, MoveDirection::Forward, 200.0);

    if (_direction == QrDirection::Left) {
        move(_hardware, _chassis, MoveDirection::Left, 995.0);
        HAL_Delay(50);
        _hardware.scanner.ReceiveDMA(
        reinterpret_cast<uint8_t *>(scannerMessage.data()), 15);
        move(_hardware, _chassis, MoveDirection::Left, 100.0);
    } else {
        move(_hardware, _chassis, MoveDirection::Right, 895.0);
        HAL_Delay(50);
        _hardware.scanner.ReceiveDMA(
        reinterpret_cast<uint8_t *>(scannerMessage.data()), 15);
        move(_hardware, _chassis, MoveDirection::Right, 100.0);
    }

    // _hardware.scanner.ReceiveDMA(
    //     reinterpret_cast<uint8_t *>(scannerMessage.data()), 15);

    while (scannerMessage[14] == 0) {
        _hardware.scanner.ReceiveDMA(
        reinterpret_cast<uint8_t *>(scannerMessage.data()), 15);
    };

    HAL_Delay(100);

    std::sscanf(
        scannerMessage.data(),
        "%1d%1d%1d+%1d%1d%1d+%1d%1d%1d+%1d%1d%1d",
        &missionData[0],
        &missionData[1],
        &missionData[2],
        &missionData[3],
        &missionData[4],
        &missionData[5],
        &missionData[6],
        &missionData[7],
        &missionData[8],
        &missionData[9],
        &missionData[10],
        &missionData[11]);

    HAL_Delay(100);

    for (uint8_t itemIndex = 0; itemIndex < itemCount; itemIndex++) {
        _mission.batches[toIndex(Batch::First)][itemIndex].color =
            static_cast<uint8_t>(missionData[itemIndex]);
        _mission.batches[toIndex(Batch::First)][itemIndex].roughSlot =
            static_cast<uint8_t>(missionData[itemIndex + 3]);
        _mission.batches[toIndex(Batch::Second)][itemIndex].color =
            static_cast<uint8_t>(missionData[itemIndex + 6]);
        _mission.batches[toIndex(Batch::Second)][itemIndex].roughSlot =
            static_cast<uint8_t>(missionData[itemIndex + 9]);
    }

    printHmiText(_hardware, scannerMessage.data());

    if (_direction == QrDirection::Left) {
        move(_hardware, _chassis, MoveDirection::Right, 985.0);
    } else {
        move(_hardware, _chassis, MoveDirection::Right, 948.0);
    }
}

} // namespace Program

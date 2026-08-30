#include "Stages.hpp"

#include "RobotControl.hpp"

#include "main.h"

namespace Program {

void runHomeStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const QrDirection _direction)
{
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(50);

    if (_direction == QrDirection::Left) {
        move(_hardware, _chassis, MoveDirection::Backward, 970.0);
        move(_hardware, _chassis, MoveDirection::Left, 1970.0);
        move(_hardware, _chassis, MoveDirection::Backward, 45.0);
    } else {
        move(_hardware, _chassis, MoveDirection::Forward, 1100.0);
        move(_hardware, _chassis, MoveDirection::Left, 1970.0);
    }

    printHmi(_hardware, "t0.txt=\"%s\"\xff\xff\xff","正确抓取：3");
    printHmi(_hardware, "t2.txt=\"%s\"\xff\xff\xff", "正确放置：3");
}

} // namespace Program

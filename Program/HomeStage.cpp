#include "Stages.hpp"

#include "RobotControl.hpp"

#include "main.h"

namespace Program {

void runHomeStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis)
{
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(50);
    move(_chassis, MoveDirection::Backward, 970.0);
    move(_chassis, MoveDirection::Left, 1970.0);
    move(_chassis, MoveDirection::Backward, 45.0);
}

} // namespace Program

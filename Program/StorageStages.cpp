#include "Stages.hpp"

#include "CameraAlignment.hpp"
#include "RobotControl.hpp"

#include "main.h"

namespace Program {
namespace {

void moveToRoughSlot(
    Rs485Chassis &_chassis,
    const uint8_t _targetSlot,
    uint8_t &_currentSlot,
    const double _slotPitchMm)
{
    const int8_t delta =
        static_cast<int8_t>(_targetSlot) -
        static_cast<int8_t>(_currentSlot);

    switch (delta) {
        case -2:
            move(_chassis, MoveDirection::Forward, 2.0 * _slotPitchMm);
            break;
        case -1:
            move(_chassis, MoveDirection::Forward, _slotPitchMm);
            break;
        case 0:
            break;
        case 1:
            move(_chassis, MoveDirection::Backward, _slotPitchMm);
            break;
        case 2:
            move(_chassis, MoveDirection::Backward, 2.0 * _slotPitchMm);
            break;
        default:
            return;
    }

    _currentSlot = _targetSlot;
}

void executeStorageTransfer(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const double _outerTravelMm,
    const AlignmentProfile &_alignment,
    const bool _isFirstItem)
{
    if (_isFirstItem) {
        _hardware.gimbal.SetCompare(gimbalGrabCompare);
        HAL_Delay(1500);
    }

    moveElevation(_hardware, _outerTravelMm, ElevationDirection::Down);
    HAL_Delay(800);
    alignRoughWithCamera(_hardware, _chassis, _alignment);
    moveElevation(_hardware, _outerTravelMm, ElevationDirection::Up);
    HAL_Delay(2000);
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(1500);
    moveElevation(_hardware, 7.15, ElevationDirection::Down);
    _hardware.arm.SetCompare(armGrabCompare);
    HAL_Delay(500);
    moveElevation(_hardware, 7.15, ElevationDirection::Up);
    _hardware.gimbal.SetCompare(gimbalGrabCompare);
    HAL_Delay(1500);
    moveElevation(_hardware, _outerTravelMm, ElevationDirection::Down);
    HAL_Delay(500);
    _hardware.arm.SetCompare(armPlaceCompare);
    HAL_Delay(500);
    moveElevation(_hardware, _outerTravelMm, ElevationDirection::Up);
}

void executeReplaceTransfer(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis)
{
    constexpr AlignmentProfile alignment{
        highXRate,
        highYRate,
        300,
        100,
    };

    alignRawWithCamera(_hardware, _chassis, alignment);
    HAL_Delay(800);
    moveElevation(_hardware, 42.9, ElevationDirection::Down);
    HAL_Delay(500);
    _hardware.arm.SetCompare(armGrabCompare);
    HAL_Delay(500);
    moveElevation(_hardware, 42.9, ElevationDirection::Up);
    HAL_Delay(500);
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(1000);
    moveElevation(_hardware, 6.435, ElevationDirection::Down);
    HAL_Delay(500);
    _hardware.arm.SetCompare(armPlaceCompare);
    HAL_Delay(500);
    moveElevation(_hardware, 6.435, ElevationDirection::Up);
    _hardware.gimbal.SetCompare(gimbalGrabCompare);
    HAL_Delay(500);
    _hardware.arm.SetCompare(armPlaceCompare);
    HAL_Delay(100);
}

} // namespace

void runRoughStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const Mission &_mission,
    const Batch _batch,
    uint8_t &_currentRoughSlot)
{
    const uint8_t batchIndex = toIndex(_batch);
    const double slotPitchMm = _batch == Batch::First ? 160.0 : 150.0;
    const AlignmentProfile alignment{
        lowXRate,
        lowYRate,
        500U,
        500U,
    };

    if (_batch == Batch::Second) {
        printHmiValue(_hardware, 62);
    }

    _hardware.gimbal.SetCompare(gimbalGrabCompare);
    HAL_Delay(20);
    _hardware.plate.SetCompare(plateCompare[0]);
    HAL_Delay(20);
    printHmiValue(_hardware, _batch == Batch::First ? 12 : 11);

    move(_chassis, MoveDirection::Rotate, -90.0);
    move(_chassis, MoveDirection::Forward, 1845.0);
    move(_chassis, MoveDirection::Rotate, -85.0);

    for (uint8_t itemIndex = 0; itemIndex < itemCount; itemIndex++) {
        _hardware.plate.SetCompare(plateCompare[itemIndex]);
        HAL_Delay(20);

        moveToRoughSlot(
            _chassis,
            _mission.batches[batchIndex][itemIndex].roughSlot,
            _currentRoughSlot,
            slotPitchMm);

        executeStorageTransfer(
            _hardware,
            _chassis,
            38.61,
            alignment,
            itemIndex == 0);
    }

    moveToRoughSlot(
        _chassis,
        roughCenterSlot,
        _currentRoughSlot,
        slotPitchMm);
}

void runReplaceStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const Mission &_mission,
    const Batch _batch,
    uint8_t &_currentRoughSlot)
{
    const uint8_t batchIndex = toIndex(_batch);
    printHmiValue(_hardware, _batch == Batch::First ? 25 : 75);

    for (uint8_t itemIndex = 0; itemIndex < itemCount; itemIndex++) {
        _hardware.plate.SetCompare(plateCompare[itemIndex]);
        HAL_Delay(20);

        moveToRoughSlot(
            _chassis,
            _mission.batches[batchIndex][itemIndex].roughSlot,
            _currentRoughSlot,
            150.0);

        executeReplaceTransfer(_hardware, _chassis);
    }

    moveToRoughSlot(
        _chassis,
        roughCenterSlot,
        _currentRoughSlot,
        150.0);
}

void runBufferStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const Mission &_mission,
    const Batch _batch,
    uint8_t &_currentRoughSlot)
{
    const uint8_t batchIndex = toIndex(_batch);
    const double outerTravelMm =
        _batch == Batch::First ? 38.61 : 20.02;
    constexpr AlignmentProfile alignment{
        lowXRate,
        lowYRate,
        1000,
        20,
    };

    move(_chassis, MoveDirection::Backward, 920.0);
    move(_chassis, MoveDirection::Rotate, 87.0);
    move(_chassis, MoveDirection::Forward, 910.0);

    for (uint8_t itemIndex = 0; itemIndex < itemCount; itemIndex++) {
        _hardware.plate.SetCompare(plateCompare[itemIndex]);
        HAL_Delay(20);

        moveToRoughSlot(
            _chassis,
            _mission.batches[batchIndex][itemIndex].roughSlot,
            _currentRoughSlot,
            150.0);

        executeStorageTransfer(
            _hardware,
            _chassis,
            outerTravelMm,
            alignment,
            itemIndex == 0);

        if (itemIndex == 2) {
            constexpr char message[] = "KO!\n";
            _hardware.camera.Send(message, sizeof(message), 100);
        }
    }

    if (_batch == Batch::Second) {
        printHmiValue(_hardware, 100);
    }

    moveToRoughSlot(
        _chassis,
        roughCenterSlot,
        _currentRoughSlot,
        150.0);
}

} // namespace Program

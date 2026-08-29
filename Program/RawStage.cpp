#include "Stages.hpp"

#include "RobotControl.hpp"

#include "main.h"

#include <cstdio>

namespace Program {
namespace {

void receiveRawMaterial(
    RobotHardware &_hardware,
    CameraBuffer &_cameraBuffer,
    CameraData &_cameraData,
    const bool _waitForMessage,
    const bool _checkColorRange)
{
    _hardware.camera.ReceiveDMA(
        reinterpret_cast<uint8_t *>(_cameraBuffer.data()), 14);

    if (_waitForMessage) {
        while (
            _cameraBuffer[0] == 0 ||
            (_checkColorRange && _cameraData[0] > 6)) {
            _hardware.camera.ReceiveDMA(
                reinterpret_cast<uint8_t *>(_cameraBuffer.data()), 14);
        }
    } else if (
        _cameraBuffer[0] == 0 ||
        (_checkColorRange && _cameraData[0] > 6)) {
        _hardware.camera.ReceiveDMA(
            reinterpret_cast<uint8_t *>(_cameraBuffer.data()), 14);
    }

    HAL_Delay(20);
    std::sscanf(
        _cameraBuffer.data(),
        "#%d,%d,%d",
        &_cameraData[0],
        &_cameraData[1],
        &_cameraData[2]);
}

void pickRawMaterial(
    RobotHardware &_hardware,
    const bool _isLastItem,
    const bool _delayAfterAcknowledgement)
{
    moveElevation(_hardware, 21.45, ElevationDirection::Down);
    HAL_Delay(100);
    _hardware.arm.SetCompare(armGrabCompare);
    HAL_Delay(1000);
    moveElevation(_hardware, 12.87, ElevationDirection::Up);
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(2000);
    _hardware.arm.SetCompare(armPlaceCompare);
    HAL_Delay(500);
    moveElevation(_hardware, 8.58, ElevationDirection::Up);
    HAL_Delay(500);

    if (!_isLastItem) {
        HAL_Delay(100);
        return;
    }

    // HAL_Delay(50);
    // constexpr char message[] = "OK!\n";
    // _hardware.camera.Send(message, sizeof(message), 100);

    // if (_delayAfterAcknowledgement) {
    //     HAL_Delay(50);
    // }
}

} // namespace

void runRawStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const Mission &_mission,
    const Batch _batch)
{
    const uint8_t batchIndex = toIndex(_batch);
    CameraBuffer cameraBuffer{};
    CameraData cameraData{7, 0, 0};

    if (_batch == Batch::First) {
        _hardware.gimbal.SetCompare(gimbalGrabCompare);
        HAL_Delay(50);
        move(_chassis, MoveDirection::Forward, 890.0);
    } else {
        printHmiValue(_hardware, 50);
        move(_chassis, MoveDirection::Backward, 970.0);
        move(_chassis, MoveDirection::Rotate, 88.0);
        move(_chassis, MoveDirection::Backward, 915.0);
    }

    for (uint8_t itemIndex = 0; itemIndex < itemCount; itemIndex++) {
        if (itemIndex == 0) {
            if (_batch == Batch::First) {
                HAL_Delay(100);
            }
            _hardware.plate.SetCompare(plateCompare[itemIndex]);
            HAL_Delay(100);
        } else {
            cameraBuffer[0] = 0;
            _hardware.gimbal.SetCompare(gimbalGrabCompare);
            HAL_Delay(_batch == Batch::First ? 20 : 50);
            _hardware.plate.SetCompare(plateCompare[itemIndex]);
            HAL_Delay(_batch == Batch::First ? 20 : 100);
        }

        do {
            receiveRawMaterial(
                _hardware,
                cameraBuffer,
                cameraData,
                itemIndex == itemCount - 1,
                _batch == Batch::First);
        } while (
            cameraData[0] !=
            _mission.batches[batchIndex][itemIndex].color);

        pickRawMaterial(
            _hardware,
            itemIndex == itemCount - 1,
            _batch == Batch::First);
    }
}

} // namespace Program

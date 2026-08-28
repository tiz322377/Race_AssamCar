#include "CameraAlignment.hpp"

#include "RobotControl.hpp"

#include "main.h"

#include <cmath>
#include <cstdio>

namespace Program {
namespace {

void correctCameraOffset(
    Rs485Chassis &_chassis,
    const CameraData &_cameraData,
    const double _xRate,
    const double _yRate)
{
    if (_cameraData[0] <= 0) {
        move(
            _chassis,
            MoveDirection::Forward,
            std::abs(static_cast<double>(_cameraData[0])) / _xRate);
    } else {
        move(
            _chassis,
            MoveDirection::Backward,
            static_cast<double>(_cameraData[0]) / _xRate);
    }

    HAL_Delay(20);

    if (_cameraData[1] <= 0) {
        move(
            _chassis,
            MoveDirection::Right,
            std::abs(static_cast<double>(_cameraData[1])) / _yRate);
    } else {
        move(
            _chassis,
            MoveDirection::Left,
            static_cast<double>(_cameraData[1]) / _yRate);
    }
}

} // namespace

void alignWithCamera(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const AlignmentProfile &_profile)
{
    CameraBuffer cameraBuffer{};
    CameraData cameraData{};

    while (cameraData[0] == 0) {
        if (_profile.firstReceiveDelayBeforeMs != 0) {
            HAL_Delay(_profile.firstReceiveDelayBeforeMs);
        }

        _hardware.camera.ReceiveDMA(
            reinterpret_cast<uint8_t *>(cameraBuffer.data()), 10);

        if (_profile.firstReceiveDelayAfterMs != 0) {
            HAL_Delay(_profile.firstReceiveDelayAfterMs);
        }

        std::sscanf(
            cameraBuffer.data(),
            "#%d,%d",
            &cameraData[0],
            &cameraData[1]);
    }

    correctCameraOffset(
        _chassis,
        cameraData,
        _profile.xRate,
        _profile.yRate);
    HAL_Delay(20);

    // _hardware.camera.ReceiveDMA(
    //     reinterpret_cast<uint8_t *>(cameraBuffer.data()), 10);
    // HAL_Delay(20);
    // std::sscanf(
    //     cameraBuffer.data(),
    //     "#%d,%d",
    //     &cameraData[0],
    //     &cameraData[1]);
    //
    // correctCameraOffset(
    //     _chassis,
    //     cameraData,
    //     _profile.xRate,
    //     _profile.yRate);
}

} // namespace Program

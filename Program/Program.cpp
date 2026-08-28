#include "Program.hpp"

#include "Peripheral/GPIO.hpp"
#include "Peripheral/Mode.hpp"
#include "Peripheral/TIM.hpp"
#include "Peripheral/Uart.hpp"
#include "Platform/Chassis/ChassisU.hpp"
#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"

#include "main.h"
#include "tim.h"
#include "usart.h"

#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

using DmaUart = Peripheral::Uart<Peripheral::DMA>;
using NormalUart = Peripheral::Uart<Peripheral::Normal>;
using NormalPwm = Peripheral::PwmChannel<Peripheral::Normal>;
using OutputPin = Peripheral::GPIOPin<Peripheral::Output>;
using MoveDirection = Platform::Chassis::MoveDirection;
using Rs485Chassis = Platform::Chassis::MecanumChassis<Platform::Chassis::RS485Bus>;

constexpr uint8_t batchCount = 2;
constexpr uint8_t itemCount = 3;
constexpr uint8_t roughCenterSlot = 2;

constexpr uint32_t gimbalGrabCompare = 72;
constexpr uint32_t gimbalPlaceCompare = 180;
constexpr uint32_t armGrabCompare = 260;
constexpr uint32_t armPlaceCompare = 220;

constexpr std::array<uint32_t, itemCount> plateCompare{75, 160, 250};

constexpr double lowXRate = 3.2;
constexpr double lowYRate = 1.6;
constexpr double highXRate = 6.4;
constexpr double highYRate = 3.2;

enum class Batch : uint8_t {
    First,
    Second,
};

enum class QrDirection : uint8_t {
    Left,
    Right,
};

enum class ElevationDirection : uint8_t {
    Up,
    Down,
};

struct Material {
    uint8_t color{};
    uint8_t roughSlot{};
};

using MaterialOrder = std::array<Material, itemCount>;

struct Mission {
    std::array<MaterialOrder, batchCount> batches{};
};

struct AlignmentProfile {
    double xRate;
    double yRate;
    uint32_t firstReceiveDelayBeforeMs;
    uint32_t firstReceiveDelayAfterMs;
};

struct RobotHardware {
    DmaUart scanner{&huart1};
    NormalUart hmi{&huart5};
    DmaUart camera{&huart3};
    DmaUart bus{&huart4};

    NormalPwm gimbal{&htim1, TIM_CHANNEL_1};
    NormalPwm plate{&htim1, TIM_CHANNEL_2};
    NormalPwm arm{&htim1, TIM_CHANNEL_3};

    OutputPin elevationEnable{GPIOA, GPIO_PIN_6};
    NormalPwm elevationPwm{&htim2, TIM_CHANNEL_1};
    OutputPin elevationDirection{GPIOA, GPIO_PIN_4};
};

using CameraBuffer = std::array<char, 15>;
using CameraData = std::array<int, 3>;

uint8_t toIndex(const Batch _batch)
{
    return static_cast<uint8_t>(_batch);
}

void move(
    Rs485Chassis &_chassis,
    const MoveDirection _direction,
    const double _distanceMm)
{
    _chassis.RunTaskTime(_direction, _distanceMm);
}

void printHmi(RobotHardware &_hardware, const char *_format, ...)
{
    std::array<char, 512> buffer{};

    va_list args;
    va_start(args, _format);
    std::vsnprintf(buffer.data(), buffer.size(), _format, args);
    va_end(args);

    _hardware.hmi.Send(
        reinterpret_cast<uint8_t *>(buffer.data()),
        static_cast<uint16_t>(std::strlen(buffer.data())),
        300);

    while (HAL_UART_GetState(&huart5) == HAL_UART_STATE_BUSY_TX) {
    }
}

void printHmiValue(RobotHardware &_hardware, const uint8_t _value)
{
    printHmi(
        _hardware,
        "j0.val=%u \xff\xff\xff",
        static_cast<unsigned int>(_value));
}

void printHmiText(RobotHardware &_hardware, const char *_text)
{
    printHmi(_hardware, "t1.txt=\"%s\"\xff\xff\xff", _text);
}

Rs485Chassis &init(RobotHardware &_hardware)
{
    constexpr double radiusMm = 37.5;
    const double motorDistanceMm = std::sqrt(105.0 * 105.0 + 105.0 * 105.0);
    constexpr std::array<uint8_t, 4> addresses{1, 2, 3, 4};
    constexpr std::array<bool, 4> directions{false, false, false, false};
    const OutputPin flowControlPin(GPIOF, GPIO_PIN_8);

    _hardware.gimbal.Start();
    _hardware.plate.Start();
    _hardware.arm.Start();

    _hardware.elevationEnable.Write(true);
    _hardware.elevationPwm.SetCompare(1000);

    Rs485Chassis &chassis = Rs485Chassis::Create(
        addresses.data(),
        &_hardware.bus,
        flowControlPin,
        16,
        directions.data(),
        true,
        radiusMm,
        motorDistanceMm);

    HAL_Delay(500);
    return chassis;
}

void resetMechanism(RobotHardware &_hardware)
{
    _hardware.plate.SetCompare(plateCompare[0]);
    HAL_Delay(50);
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(50);
    _hardware.arm.SetCompare(armPlaceCompare);
    HAL_Delay(50);
}

void moveElevation(
    RobotHardware &_hardware,
    const double _distanceMm,
    const ElevationDirection _direction)
{
    _hardware.elevationDirection.Write(_direction == ElevationDirection::Down);
    HAL_Delay(20);

    HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);

    const double steps = _distanceMm / 14.3 * 200.0;
    ElevationPulseCounterReset(static_cast<uint32_t>(steps));

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

    while (!ElevationPulseCounterIsCompleted()) {
    }
}

void scanMission(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    Mission &_mission,
    const QrDirection _direction)
{
    std::array<char, 16> scannerMessage{};
    std::array<int, 12> missionData{};

    move(_chassis, MoveDirection::Forward, 200.0);

    if (_direction == QrDirection::Left) {
        move(_chassis, MoveDirection::Left, 995.0);
        HAL_Delay(50);
        move(_chassis, MoveDirection::Left, 100.0);
    } else {
        move(_chassis, MoveDirection::Right, 895.0);
        HAL_Delay(50);
        move(_chassis, MoveDirection::Right, 100.0);
    }

    _hardware.scanner.ReceiveDMA(
        reinterpret_cast<uint8_t *>(scannerMessage.data()), 15);

    while (scannerMessage[14] == 0) {
    }

    HAL_Delay(50);

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
        move(_chassis, MoveDirection::Right, 990.0);
    } else {
        move(_chassis, MoveDirection::Right, 820.0);
    }
}

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
    moveElevation(_hardware, 14.3, ElevationDirection::Up);
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(2000);
    _hardware.arm.SetCompare(armPlaceCompare);
    moveElevation(_hardware, 7.15, ElevationDirection::Up);

    if (!_isLastItem) {
        HAL_Delay(100);
        return;
    }

    HAL_Delay(50);
    constexpr char message[] = "OK!\n";
    _hardware.camera.Send(message, sizeof(message), 100);

    if (_delayAfterAcknowledgement) {
        HAL_Delay(50);
    }
}

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
            std::abs(static_cast<double>(_cameraData[1])) / _yRate + 3.0);
    } else {
        move(
            _chassis,
            MoveDirection::Left,
            static_cast<double>(_cameraData[1]) / _yRate - 3.0);
    }
}

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

    _hardware.camera.ReceiveDMA(
        reinterpret_cast<uint8_t *>(cameraBuffer.data()), 10);
    HAL_Delay(20);
    std::sscanf(
        cameraBuffer.data(),
        "#%d,%d",
        &cameraData[0],
        &cameraData[1]);

    correctCameraOffset(
        _chassis,
        cameraData,
        _profile.xRate,
        _profile.yRate);
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
    alignWithCamera(_hardware, _chassis, _alignment);
    moveElevation(_hardware, _outerTravelMm, ElevationDirection::Up);
    HAL_Delay(2000);
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(1500);
    moveElevation(_hardware, 14.3, ElevationDirection::Down);
    _hardware.arm.SetCompare(armGrabCompare);
    HAL_Delay(1500);
    moveElevation(_hardware, 14.3, ElevationDirection::Up);
    _hardware.gimbal.SetCompare(gimbalGrabCompare);
    HAL_Delay(1500);
    moveElevation(_hardware, _outerTravelMm, ElevationDirection::Down);
    HAL_Delay(20);
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
        0,
        100,
    };

    alignWithCamera(_hardware, _chassis, alignment);
    moveElevation(_hardware, 42.9, ElevationDirection::Down);
    HAL_Delay(20);
    _hardware.arm.SetCompare(armGrabCompare);
    HAL_Delay(20);
    moveElevation(_hardware, 42.9, ElevationDirection::Up);
    HAL_Delay(20);
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(1000);
    moveElevation(_hardware, 7.15, ElevationDirection::Down);
    _hardware.arm.SetCompare(armPlaceCompare);
    HAL_Delay(20);
    moveElevation(_hardware, 7.15, ElevationDirection::Up);
    _hardware.gimbal.SetCompare(gimbalGrabCompare);
    HAL_Delay(20);
    _hardware.arm.SetCompare(armPlaceCompare);
    HAL_Delay(20);
}

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
        _batch == Batch::First ? 1000U : 0U,
        _batch == Batch::First ? 20U : 100U,
    };

    if (_batch == Batch::Second) {
        printHmiValue(_hardware, 62);
    }

    _hardware.gimbal.SetCompare(gimbalGrabCompare);
    HAL_Delay(20);
    _hardware.plate.SetCompare(plateCompare[0]);
    HAL_Delay(20);
    printHmiValue(_hardware, _batch == Batch::First ? 12 : 11);

    move(_chassis, MoveDirection::Left, 1835.0);
    move(_chassis, MoveDirection::Rotate, 175.0);

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
            42.9,
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
        _batch == Batch::First ? 42.9 : 20.02;
    constexpr AlignmentProfile alignment{
        lowXRate,
        lowYRate,
        1000,
        20,
    };

    if (_batch == Batch::First) {
        printHmiValue(_hardware, 37);
        move(_chassis, MoveDirection::Backward, 920.0);
        move(_chassis, MoveDirection::Left, 880.0);
    } else {
        printHmiValue(_hardware, 87);
        move(_chassis, MoveDirection::Backward, 830.0);
        move(_chassis, MoveDirection::Left, 1015.0);
    }

    move(_chassis, MoveDirection::Rotate, 88.0);

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

void runHomeStage(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis)
{
    _hardware.gimbal.SetCompare(gimbalPlaceCompare);
    HAL_Delay(50);
    move(_chassis, MoveDirection::Backward, 970.0);
    move(_chassis, MoveDirection::Left, 1850.0);
    move(_chassis, MoveDirection::Backward, 115.0);
}

} // namespace

extern "C" [[noreturn]] void Main()
{
    RobotHardware hardware;
    Mission mission;
    uint8_t currentRoughSlot = roughCenterSlot;

    Rs485Chassis &chassis = init(hardware);
    resetMechanism(hardware);
    scanMission(hardware, chassis, mission, QrDirection::Left);

    runRawStage(hardware, chassis, mission, Batch::First);
    runRoughStage(hardware, chassis, mission, Batch::First, currentRoughSlot);
    runReplaceStage(hardware, chassis, mission, Batch::First, currentRoughSlot);
    runBufferStage(hardware, chassis, mission, Batch::First, currentRoughSlot);

    runRawStage(hardware, chassis, mission, Batch::Second);
    runRoughStage(hardware, chassis, mission, Batch::Second, currentRoughSlot);
    runReplaceStage(hardware, chassis, mission, Batch::Second, currentRoughSlot);
    runBufferStage(hardware, chassis, mission, Batch::Second, currentRoughSlot);

    runHomeStage(hardware, chassis);

    for (;;) {
    }
}

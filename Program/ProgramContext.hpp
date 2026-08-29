#ifndef PROGRAM_CONTEXT_HPP
#define PROGRAM_CONTEXT_HPP

#include "Peripheral/GPIO.hpp"
#include "Peripheral/Mode.hpp"
#include "Peripheral/TIM.hpp"
#include "Peripheral/Uart.hpp"
#include "Platform/Chassis/ChassisU.hpp"
#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"

#include <array>
#include <cstdint>

namespace Program {

using DmaUart = Peripheral::Uart<Peripheral::DMA>;
using NormalUart = Peripheral::Uart<Peripheral::Normal>;
using NormalPwm = Peripheral::PwmChannel<Peripheral::Normal>;
using OutputPin = Peripheral::GPIOPin<Peripheral::Output>;
using MoveDirection = Platform::Chassis::MoveDirection;
using Rs485Chassis =
    Platform::Chassis::MecanumChassis<Platform::Chassis::RS485Bus>;

inline constexpr uint8_t batchCount = 2;
inline constexpr uint8_t itemCount = 3;
inline constexpr uint8_t roughCenterSlot = 2;

inline constexpr uint32_t gimbalGrabCompare = 72;
inline constexpr uint32_t gimbalPlaceCompare = 181;
inline constexpr uint32_t armGrabCompare = 260;
inline constexpr uint32_t armPlaceCompare = 220;

inline constexpr std::array<uint32_t, itemCount> plateCompare{72, 155, 245};

inline constexpr double lowXRate = 3.45;
inline constexpr double lowYRate = 4.8;
inline constexpr double highXRate = 6.4;
inline constexpr double highYRate = 3.2;
inline constexpr double rawXRate = 3.33;
inline constexpr double rawYRate = 3.95;

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
    RobotHardware();

    DmaUart scanner;
    NormalUart hmi;
    DmaUart camera;
    DmaUart bus;

    NormalPwm gimbal;
    NormalPwm plate;
    NormalPwm arm;

    OutputPin elevationEnable;
    NormalPwm elevationPwm;
    OutputPin elevationDirection;
};

using CameraBuffer = std::array<char, 13>;
using CameraData = std::array<int, 3>;

constexpr uint8_t toIndex(const Batch _batch)
{
    return static_cast<uint8_t>(_batch);
}

} // namespace Program

#endif // PROGRAM_CONTEXT_HPP

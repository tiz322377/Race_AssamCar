//
// Created by Administrator on 25-7-3.
//

#ifndef PROGRAM_HPP
#define PROGRAM_HPP


#include "Config/Config.hpp"
#include "Peripheral/GPIO.hpp"
#include "Peripheral/uart.hpp"
#include "Peripheral/TIM.hpp"

#include <cstdint>

enum class RobotState : uint8_t {
    Zero,

    GoQr,
    WaitQr,
    ParseQr,

    GoRaw,
    GrabRaw,

    GoRough,
    PlaceRough,
    ReloadRough,

    GoBuffer,
    PlaceBuffer,

    ReturnStart,
    Finished,
    Fault
};

struct Mission {
    // batchColor[批次][搬运顺序]
    uint8_t batchColor[2][3]{};

    // roughSlot[批次][搬运顺序]
    uint8_t roughSlot[2][3]{};
};

struct RobotContext {
    RobotState state = RobotState::Zero;
    Mission mission{};

    uint8_t batch = 0;   // 0：第一批，1：第二批
    uint8_t item  = 0;   // 当前处理第几个物料

    // 记录第一批每种颜色在暂存区的位置
    // 下标为颜色1～6
    uint8_t bufferSlotByColor[7]{};
};

static RobotContext robot;

void print(const char*format,...);

#endif //PROGRAM_HPP

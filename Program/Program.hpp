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
#include <cstdbool>

extern volatile uint32_t PULSE_TARGET;  // 目标脉冲数
extern volatile uint32_t PULSE_COUNT;   // 当前已发脉冲数
extern volatile bool PULSE_COMPLETED; //已完成标志

enum Dir : uint8_t
{
    Up,
    Down,
};

enum class RobotState : uint8_t {
    Zero,

    Qr,

    Raw1,
    Raw2,

    Rough1,
    Rough2,

    Replace1,
    Replace2,

    Buffer1,
    Buffer2,

    Finish,
};

enum class ActionState : uint8_t{
    Start,

    Go,
    Wait,
    Parse,

    First,
    Second,
    Thrid,

    Finish,
};

struct Mission {
    // batchColor[批次][搬运顺序]
    uint8_t batchColor[2][3]{};

    // roughSlot[批次][搬运顺序]
    uint8_t roughSlot[2][3]{};
};

struct RobotContext {
    RobotState state = RobotState::Zero;
    ActionState step = ActionState::Start;
    Mission mission{};

    uint8_t batch = 0;   // 0：第一批，1：第二批
    uint8_t item  = 0;   // 当前处理第几个物料

    // 记录第一批每种颜色在暂存区的位置
    // 下标为颜色1～6
    uint8_t bufferSlotByColor[7]{};
};

static RobotContext robot;

void RobotEvent();
void QrEvent(uint8_t dir);
void Row1Event();
void Rough1Event();
void Replace1Event();
void Buffer1Event();
void Row2Event();
void Rough2Event();
void Replace2Event();
void Buffer2Event();
void print(const char*format,...);
void Elevation_Move(double distance,Dir dir);

#endif //PROGRAM_HPP

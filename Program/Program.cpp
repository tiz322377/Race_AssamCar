//
// Created by Administrator on 25-7-3.
//

#include "Program.hpp"


#include "Config/Config.hpp"
#include "Peripheral/Mode.hpp"
#include "Peripheral/TIM.hpp"
#include "Platform/Chassis/ChassisU.hpp"
#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"

#include "tim.h"
#include "usart.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdarg>

using namespace Peripheral;
using namespace Platform::Chassis;

const auto bus = Uart<DMA>(&huart4);
const auto gm65 = Uart<DMA>(&huart1);//9600
const auto HMI = Uart<Normal>(&huart2);
const auto gimbal = PwmChannel<Normal>(&htim1,TIM_CHANNEL_1);
const auto plate = PwmChannel<Normal>(&htim1,TIM_CHANNEL_2);
const auto arm = PwmChannel<Normal>(&htim2,TIM_CHANNEL_1);

char HMI_txtcontrol[7] = {"t1.txt"};
char scaner_message[15] = {0};
uint8_t mission_message[12] = {0};

constexpr auto GIMBAL_GRAB = 75;
constexpr auto GIMBAL_PLACE = 181;
constexpr auto ARM_GRAB = 131;
constexpr auto ARM_PLACE = 100;
constexpr auto PLATE_FRIST = 50;
constexpr auto PLATE_SECOND =138;
constexpr auto PLATE_THIRD = 228;

void Init()
{
    constexpr auto radius = 37.5;
    const auto distance = std::sqrt(105 * 105 + 105 * 105);
    constexpr uint8_t address[4] = {1,2,3,4};
    constexpr bool dirs[4] = {false,false,false,false};
    const GPIOPin<Output> flowControlPin(GPIOF,GPIO_PIN_8);
    robot.state = RobotState::Zero;

    gimbal.Start();
    plate.Start();
    arm.Start();

    MecanumChassis<RS485Bus>::Create(
        address, &bus, flowControlPin, 16, dirs, true, radius, distance);

    HAL_Delay(500);
}

extern "C" [[noreturn]] void Main()
{

    Init();

    // MCRSBPtr->RunTaskTime(MoveDirection::Right,1065.0f);//x85y0
    // MCRSBPtr->RunTaskTime(MoveDirection::Forward,995.0f);
    // HAL_Delay(50);
    // MCRSBPtr->RunTaskTime(MoveDirection::Forward,200.0f);

    for (;;) {
        switch (robot.state) {
            case RobotState::Zero:
                plate.SetCompare(PLATE_FRIST);
                HAL_Delay(50);
                gimbal.SetCompare(GIMBAL_PLACE);
                HAL_Delay(50);
                arm.SetCompare(ARM_PLACE);
                HAL_Delay(50);
                robot.state = RobotState::GoQr;
                break;

            case RobotState::GoQr:
                MCRSBPtr->RunTaskTime(MoveDirection::Left,950.0f);
                HAL_Delay(50);
                MCRSBPtr->RunTaskTime(MoveDirection::Left,200.0f);//x1150y0
                gm65.ReceiveDMA((uint8_t *)scaner_message,15);
                while(HAL_UART_GetState(&huart1)==HAL_UART_STATE_BUSY_RX);
                robot.state =RobotState::WaitQr;
                break;

            case RobotState::WaitQr:
                sscanf(scaner_message,"%1d%1d%1d+%1d%1d%1d+%1d%1d%1d+%1d%1d%1d",mission_message+0,mission_message+1,
                        mission_message+2,mission_message+3,mission_message+4,mission_message+5,mission_message+6,mission_message+7,mission_message+8,
                        mission_message+9,mission_message+10,mission_message+11);
                for (uint8_t i = 0; i < 3; i++) {
                    robot.mission.batchColor[0][i] = mission_message[i];
                    robot.mission.roughSlot[0][i] = mission_message[i+3];
                    robot.mission.batchColor[1][i] = mission_message[i+6];
                    robot.mission.roughSlot[1][i] = mission_message[i+9];
                }
                robot.state = RobotState::ParseQr;
                break;

            case RobotState::ParseQr:
                print("%s=\"%s\"\xff\xff\xff",HMI_txtcontrol,scaner_message);
                robot.state = RobotState::GoRaw;
                break;
        }


    }
}

void print(const char*format,...){
    char buf[512];
    va_list args;
    va_start(args,format);
    vsnprintf(buf,sizeof(buf),format,args);
    va_end(args);

    HAL_UART_Transmit(&huart2,(uint8_t*)buf,strlen(buf),200);

    while(HAL_UART_GetState(&huart2)==HAL_UART_STATE_BUSY_TX);
}

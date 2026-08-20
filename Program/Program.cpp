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

const auto gm65 = Uart<DMA>(&huart1);//9600
const auto HMI = Uart<Normal>(&huart2);
const auto camera = Uart<DMA>(&huart3);
const auto bus = Uart<DMA>(&huart4);
const auto gimbal = PwmChannel<Normal>(&htim1,TIM_CHANNEL_1);
const auto plate = PwmChannel<Normal>(&htim1,TIM_CHANNEL_2);
const auto arm = PwmChannel<Normal>(&htim1,TIM_CHANNEL_3);
const auto en_elevation = GPIOPin<Output>(GPIOA,GPIO_PIN_6);
const auto step_elevation = PwmChannel<Normal>(&htim2,TIM_CHANNEL_1);
const auto dir_elevation = GPIOPin<Output>(GPIOA,GPIO_PIN_4);

char HMI_txtcontrol[7] = {"t1.txt"};
char scanner_message[15] = {0};
char camera_message[14] = {0};
uint8_t mission_data[12] = {0};
int8_t camera_data[3] = {0};

volatile uint32_t PULSE_TARGET = 0;
volatile uint32_t PULSE_COUNT = 0;
volatile bool PULSE_COMPLETED = false;
constexpr auto GIMBAL_GRAB = 75;
constexpr auto GIMBAL_PLACE = 181;
constexpr auto ARM_GRAB = 250;
constexpr auto ARM_PLACE = 220;
constexpr auto PLATE_FIRST = 50;
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

    en_elevation.Write(GPIO_PIN_SET);
    step_elevation.SetCompare(1000);

    MecanumChassis<RS485Bus>::Create(
        address, &bus, flowControlPin, 16, dirs, true, radius, distance);

    HAL_Delay(500);
}

extern "C" [[noreturn]] void Main()
{

    Init();

    for (;;) {
        switch (robot.state) {
            case RobotState::Zero:
                plate.SetCompare(PLATE_FIRST);
                HAL_Delay(50);
                gimbal.SetCompare(GIMBAL_PLACE);
                HAL_Delay(50);
                arm.SetCompare(ARM_PLACE);
                HAL_Delay(50);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,100.0f);//x0y100
                robot.state = RobotState::GoQr;
                break;

            case RobotState::GoQr:
                MCRSBPtr->RunTaskTime(MoveDirection::Left,950.0f);
                HAL_Delay(50);
                MCRSBPtr->RunTaskTime(MoveDirection::Left,250.0f);//x1200y100
                gm65.ReceiveDMA((uint8_t *)scanner_message,15);
                while(HAL_UART_GetState(&huart1)==HAL_UART_STATE_BUSY_RX);
                robot.state =RobotState::WaitQr;
                break;

            case RobotState::WaitQr:
                sscanf(scanner_message,"%1d%1d%1d+%1d%1d%1d+%1d%1d%1d+%1d%1d%1d",mission_data+0,mission_data+1,
                        mission_data+2,mission_data+3,mission_data+4,mission_data+5,mission_data+6,mission_data+7,mission_data+8,
                        mission_data+9,mission_data+10,mission_data+11);
                for (uint8_t i = 0; i < 3; i++) {
                    robot.mission.batchColor[0][i] = mission_data[i];
                    robot.mission.roughSlot[0][i] = mission_data[i+3];
                    robot.mission.batchColor[1][i] = mission_data[i+6];
                    robot.mission.roughSlot[1][i] = mission_data[i+9];
                }
                robot.state = RobotState::ParseQr;
                break;

            case RobotState::ParseQr:
                print("%s=\"%s\"\xff\xff\xff",HMI_txtcontrol,scanner_message);
                robot.state = RobotState::GoRaw;
                break;

            case RobotState::GoRaw:
                MCRSBPtr->RunTaskTime(MoveDirection::Right,1000.0f);//x200y100
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,1100.0f);//x1200y1200
                robot.state = RobotState::GrabRaw;

            case RobotState::GrabRaw:
                gimbal.SetCompare(GIMBAL_GRAB);
                HAL_Delay(50);
                Elevation_Move(28.6,Down);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(50);
                sscanf(camera_message,"#%hhd,%hhd,%hhd",camera_data+0,camera_data+1,camera_data+2);
                if (camera_data[0] == robot.mission.batchColor[0][0]) {
                    arm.SetCompare(ARM_GRAB);
                    HAL_Delay(50);
                    Elevation_Move(14.3,Up);
                    while (PULSE_COMPLETED == true) {
                        gimbal.SetCompare(GIMBAL_PLACE);
                        HAL_Delay(50);
                        arm.SetCompare(ARM_PLACE);
                        HAL_Delay(50);
                        plate.SetCompare(PLATE_SECOND);
                        HAL_Delay(50);
                    }
                }
        }


    }
}

// void RobotEvent()
// {
//     switch (robot.state) {
//         case RobotState::Zero:
//             plate.SetCompare(PLATE_FIRST);
//             HAL_Delay(50);
//             gimbal.SetCompare(GIMBAL_PLACE);
//             HAL_Delay(50);
//             arm.SetCompare(ARM_PLACE);
//             HAL_Delay(50);
//             MCRSBPtr->RunTaskTime(MoveDirection::Forward,100.0f);//x0y100
//             robot.state = RobotState::Qr;
//             break;
//         case
// }
//
// void QrEvent(){
//         switch (robot.step) {
//             case ActionState::Enter:
//                 robot.step = ActionState::Go;
//                 break;
//             case ActionState::Go:
//
//         }
// }

void print(const char*format,...){
    char buf[512];
    va_list args;
    va_start(args,format);
    vsnprintf(buf,sizeof(buf),format,args);
    va_end(args);

    HAL_UART_Transmit(&huart2,(uint8_t*)buf,strlen(buf),200);

    while(HAL_UART_GetState(&huart2)==HAL_UART_STATE_BUSY_TX);
}

void Elevation_Move(double distance,Dir dir)
{
    if (dir == Up) {
        dir_elevation.Write(GPIO_PIN_RESET);//UP
    }
    else if (dir == Down) {
        dir_elevation.Write(GPIO_PIN_SET);//Down
    }

    HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);

    uint32_t steps =  distance / 14.3 * 3200;

    PULSE_COUNT  = 0;
    PULSE_TARGET = steps;
    PULSE_COMPLETED = false;

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
}
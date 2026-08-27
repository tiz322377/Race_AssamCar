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
const auto HMI = Uart<Normal>(&huart5);
const auto camera = Uart<DMA>(&huart3);
const auto bus = Uart<DMA>(&huart4);
const auto gimbal = PwmChannel<Normal>(&htim1,TIM_CHANNEL_1);
const auto plate = PwmChannel<Normal>(&htim1,TIM_CHANNEL_2);
const auto arm = PwmChannel<Normal>(&htim1,TIM_CHANNEL_3);
const auto en_elevation = GPIOPin<Output>(GPIOA,GPIO_PIN_6);
const auto step_elevation = PwmChannel<Normal>(&htim2,TIM_CHANNEL_1);
const auto dir_elevation = GPIOPin<Output>(GPIOA,GPIO_PIN_4);

char HMI_txtcontrol[7] = {"t1.txt"};
char HMI_valcontrol[7] = {"j0.val"};
char scanner_message[16] = {0};
char camera_message[15] = {0};
int mission_data[12] = {0};
int camera_data[3] = {7};

volatile uint32_t PULSE_TARGET = 0;
volatile uint32_t PULSE_COUNT = 0;
volatile bool PULSE_COMPLETED = false;
constexpr auto GIMBAL_GRAB = 72;
constexpr auto GIMBAL_PLACE = 180;
constexpr auto ARM_GRAB = 260;
constexpr auto ARM_PLACE = 220;
uint8_t PLATE_MEMORY[3] = {75,160,250};
int8_t Ring_Rough = 2;

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
    //Rotate顺时针
    Init();

    // robot.mission.batchColor[0][0] = 4;
    // robot.mission.batchColor[0][1] = 5;
    // robot.mission.batchColor[0][2] = 6;
    // robot.state = RobotState::Raw2;
    // robot.step = ActionState::GoFirst;

    for (;;) {
        RobotEvent();
    }
}

void RobotEvent(){
    switch (robot.state) {
        case RobotState::Zero:
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(50);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(50);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(50);
            robot.state = RobotState::Qr;
            break;

        case RobotState::Qr:
            QrEvent(1);
            if (robot.step == ActionState::Finish) {
                robot.step = ActionState::Start;
                robot.state = RobotState::Raw1;
            }
            break;

        case RobotState::Raw1:
            Row1Event();
            if (robot.step == ActionState::Finish) {
                robot.step = ActionState::Start;
                robot.state = RobotState::Rough1;
            }
            break;

        case RobotState::Rough1:
            Rough1Event();
            if (robot.step == ActionState::Finish) {
                robot.step = ActionState::Start;
                robot.state = RobotState::Replace1;
            }
            break;

        case RobotState::Replace1:
            Replace1Event();
            if (robot.step == ActionState::Finish) {
                robot.step = ActionState::Start;
                robot.state = RobotState::Buffer1;
            }
            break;

        case RobotState::Buffer1:
            Buffer1Event();
            if (robot.step == ActionState::Finish) {
                robot.step = ActionState::Start;
                robot.state = RobotState::Raw2;
            }
            break;

        case RobotState::Raw2:
            Row2Event();
            if (robot.step == ActionState::Finish) {
                robot.step = ActionState::Start;
                robot.state = RobotState::Rough2;
            }
            break;

        case RobotState::Rough2:
            Rough2Event();
            if (robot.step == ActionState::Finish) {
                robot.step = ActionState::Start;
                robot.state = RobotState::Replace2;
            }
            break;

        case RobotState::Replace2:
            Replace2Event();
            if (robot.step == ActionState::Finish) {
                robot.step = ActionState::Start;
                robot.state = RobotState::Buffer2;
            }
            break;

        case RobotState::Buffer2:
            Buffer2Event();
            if (robot.step == ActionState::Finish) {
                robot.state = RobotState::Finish;
            }
            break;

        default:
            break;
    }
}
//dir == 1 or 2
void QrEvent(uint8_t dir){
        switch (robot.step) {
            case ActionState::Start:
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,200.0f);//x0y200
                robot.step = ActionState::Go;
                break;

            case ActionState::Go:
                if (dir == 1) {
                    MCRSBPtr->RunTaskTime(MoveDirection::Left,995.0f);//x1100y200
                    HAL_Delay(50);
                    MCRSBPtr->RunTaskTime(MoveDirection::Left,100.0f);//x1200y200
                    gm65.ReceiveDMA((uint8_t *)scanner_message,15);
                    while (scanner_message[14] == 0){};
                    HAL_Delay(50);
                    sscanf(scanner_message,"%1d%1d%1d+%1d%1d%1d+%1d%1d%1d+%1d%1d%1d",
                        mission_data+0,mission_data+1,mission_data+2,mission_data+3,mission_data+4,mission_data+5,
                        mission_data+6,mission_data+7,mission_data+8,mission_data+9,mission_data+10,mission_data+11);
                    HAL_Delay(50);
                    robot.step = ActionState::Wait;
                    break;
                }
                else if (dir == 2) {
                    MCRSBPtr->RunTaskTime(MoveDirection::Right,895.0f);//x1300y200
                    HAL_Delay(50);
                    MCRSBPtr->RunTaskTime(MoveDirection::Right,100.0f);//x1200y20
                    gm65.ReceiveDMA((uint8_t *)scanner_message,15);
                    while (scanner_message[14] == 0){};
                    HAL_Delay(50);
                    sscanf(scanner_message,"%1d%1d%1d+%1d%1d%1d+%1d%1d%1d+%1d%1d%1d",mission_data+0,mission_data+1,
                            mission_data+2,mission_data+3,mission_data+4,mission_data+5,mission_data+6,mission_data+7,mission_data+8,
                            mission_data+9,mission_data+10,mission_data+11);
                    HAL_Delay(50);
                    robot.step = ActionState::Wait;
                    break;
                }

            case ActionState::Wait:
                HAL_Delay(50);
                for (uint8_t i = 0; i < 3; i++) {
                    robot.mission.batchColor[0][i] = mission_data[i];
                    robot.mission.roughSlot[0][i] = mission_data[i+3];
                    robot.mission.batchColor[1][i] = mission_data[i+6];
                    robot.mission.roughSlot[1][i] = mission_data[i+9];
                }
                robot.step = ActionState::Parse;
                break;

            case ActionState::Parse:
                print("%s=\"%s\"\xff\xff\xff",HMI_txtcontrol,scanner_message);
                if (dir == 1) {
                    MCRSBPtr->RunTaskTime(MoveDirection::Right,990.0f);//x250y200
                }
                else if (dir == 2) {
                    MCRSBPtr->RunTaskTime(MoveDirection::Right,820.0f);//x250y200
                }
                robot.step = ActionState::Finish;
                break;

            default:
                robot.step = ActionState::Finish;
                break;
        }
}

void Row1Event(){
        switch (robot.step) {
            case ActionState::Start://x250y200
                gimbal.SetCompare(GIMBAL_GRAB);
                HAL_Delay(50);
                robot.step = ActionState::Go;
                break;

            case ActionState::Go:
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,890.0f);//x250y1090
                robot.step = ActionState::GoFirst;
                break;

            case ActionState::GoFirst:
                HAL_Delay(100);
                plate.SetCompare(PLATE_MEMORY[0]);
                HAL_Delay(100);
                robot.step = ActionState::First;
                break;

            case ActionState::First:
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                if (camera_message[0] == 0 || camera_data[0] > 6) {
                    camera.ReceiveDMA((uint8_t *)camera_message,14);
                };
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d,%d",camera_data+0,camera_data+1,camera_data+2);
                if (camera_data[0] == robot.mission.batchColor[0][0]) {
                    Elevation_Move(21.45,Down);
                    while (PULSE_COMPLETED != true){};
                    HAL_Delay(100);
                    arm.SetCompare(ARM_GRAB);
                    HAL_Delay(1000);
                    Elevation_Move(14.3,Up);
                    while (PULSE_COMPLETED != true){};
                    gimbal.SetCompare(GIMBAL_PLACE);
                    HAL_Delay(2000);
                    arm.SetCompare(ARM_PLACE);
                    Elevation_Move(7.15,Up);
                    while (PULSE_COMPLETED != true){};
                    HAL_Delay(100);
                    robot.step = ActionState::GoSecond;
                }
                break;

            case ActionState::GoSecond:
                camera_message[0] = 0;
                gimbal.SetCompare(GIMBAL_GRAB);
                HAL_Delay(20);
                plate.SetCompare(PLATE_MEMORY[1]);
                HAL_Delay(20);
                robot.step = ActionState::Second;
                break;

            case ActionState::Second:
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                if (camera_message[0] == 0 || camera_data[0] > 6) {
                    camera.ReceiveDMA((uint8_t *)camera_message,14);
                };
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d,%d",camera_data+0,camera_data+1,camera_data+2);
                if (camera_data[0] == robot.mission.batchColor[0][1]) {
                    Elevation_Move(21.45,Down);
                    while (PULSE_COMPLETED != true){};
                    HAL_Delay(100);
                    arm.SetCompare(ARM_GRAB);
                    HAL_Delay(1000);
                    Elevation_Move(14.3,Up);
                    while (PULSE_COMPLETED != true){};
                    gimbal.SetCompare(GIMBAL_PLACE);
                    HAL_Delay(2000);
                    arm.SetCompare(ARM_PLACE);
                    Elevation_Move(7.15,Up);
                    while (PULSE_COMPLETED != true){};
                    HAL_Delay(100);
                    robot.step = ActionState::GoThird;
                }
                break;

            case ActionState::GoThird:
                camera_message[0] = 0;
                gimbal.SetCompare(GIMBAL_GRAB);
                HAL_Delay(20);
                plate.SetCompare(PLATE_MEMORY[2]);
                HAL_Delay(20);
                robot.step = ActionState::Third;
                break;

            case ActionState::Third:
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                while (camera_message[0] == 0 || camera_data[0] > 6) {
                    camera.ReceiveDMA((uint8_t *)camera_message,14);
                };
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d,%d",camera_data+0,camera_data+1,camera_data+2);
                if (camera_data[0] == robot.mission.batchColor[0][2]) {
                    Elevation_Move(21.45,Down);
                    while (PULSE_COMPLETED != true){};
                    HAL_Delay(100);
                    arm.SetCompare(ARM_GRAB);
                    HAL_Delay(1000);
                    Elevation_Move(14.3,Up);
                    while (PULSE_COMPLETED != true){};
                    gimbal.SetCompare(GIMBAL_PLACE);
                    HAL_Delay(2000);
                    arm.SetCompare(ARM_PLACE);
                    Elevation_Move(7.15,Up);
                    while (PULSE_COMPLETED != true){};
                    HAL_Delay(50);
                    const char message[4] = {"OK!"};
                    camera.Send(message,sizeof(message),100);
                    HAL_Delay(50);
                    robot.step = ActionState::Finish;
                }
                break;

            default:
                robot.step = ActionState::Finish;
                break;
        }
}

void Rough1Event(){
    switch (robot.step) {
        case ActionState::Start://x250y1090
            camera_data[0] = 0;
            camera_data[1] = 0;
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(20);
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(20);
            int8_t temp;
            print("%s=%d \xff\xff\xff",HMI_valcontrol,12);
            robot.step = ActionState::Go;
            break;

        case ActionState::Go:
            MCRSBPtr->RunTaskTime(MoveDirection::Left,1835.0f);//x2085y1090
            MCRSBPtr->RunTaskTime(MoveDirection::Rotate,175.0f);//x2085y1090旋转
            robot.step = ActionState::GoFirst;
            break;

        case ActionState::GoFirst:
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[0][0] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
                }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y940
                Ring_Rough = Ring_Rough + temp;
                }
            else if (temp == 0) {
                robot.step = ActionState::First;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1240
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::First:
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                HAL_Delay(1000);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::GoSecond;
            break;

        case ActionState::GoSecond:
            plate.SetCompare(PLATE_MEMORY[1]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[0][1] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Second;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Second:
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                HAL_Delay(1000);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::GoThird;
            break;

        case ActionState::GoThird:
            plate.SetCompare(PLATE_MEMORY[2]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[0][2] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Third;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Third:
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                HAL_Delay(1000);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::Wait;
            break;

        case ActionState::Wait:
            temp = 2 - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Finish;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        default:
            robot.step = ActionState::Finish;
            break;
    }
}

void Replace1Event(){
    switch (robot.step) {
        case ActionState::Start://x2085y1070
            print("%s=%d \xff\xff\xff",HMI_valcontrol,25);
            robot.step = ActionState::GoFirst;
            break;

        case ActionState::GoFirst:
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(20);
            int8_t temp;
            temp = robot.mission.roughSlot[0][0] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::First;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::First:
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(100);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(20);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1000);
            Elevation_Move(7.15,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            Elevation_Move(7.15,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            robot.step = ActionState::GoSecond;
            break;

        case ActionState::GoSecond:
            plate.SetCompare(PLATE_MEMORY[1]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[0][1] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Second;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Second:
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(100);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(20);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1000);
            Elevation_Move(7.15,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            Elevation_Move(7.15,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            robot.step = ActionState::GoThird;
            break;

        case ActionState::GoThird:
            plate.SetCompare(PLATE_MEMORY[2]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[0][2] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Third;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;


        case ActionState::Third:
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(100);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(20);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1000);
            Elevation_Move(7.15,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            Elevation_Move(7.15,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            robot.step = ActionState::Wait;
            break;

        case ActionState::Wait:
            temp = 2 - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Finish;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        default:
            robot.step = ActionState::Finish;
            break;
    }
}

void Buffer1Event(){
    switch (robot.step) {
        case ActionState::Start://x2085y1070
            print("%s=%d \xff\xff\xff",HMI_valcontrol,37);
            MCRSBPtr->RunTaskTime(MoveDirection::Backward,920.0f);//x2085y1990
            MCRSBPtr->RunTaskTime(MoveDirection::Left,865.0f);//x1220y1990
            int8_t temp;
            robot.step = ActionState::Go;
            break;

        case ActionState::Go:
            MCRSBPtr->RunTaskTime(MoveDirection::Rotate,88.0f);//x1220y1990
            robot.step = ActionState::GoFirst;
            break;

        case ActionState::GoFirst:
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[0][0] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::First;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::First:
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                HAL_Delay(1000);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::GoSecond;
            break;

        case ActionState::GoSecond:
            plate.SetCompare(PLATE_MEMORY[1]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[0][1] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Second;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Second:
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                HAL_Delay(1000);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::GoThird;
            break;

        case ActionState::GoThird:
            plate.SetCompare(PLATE_MEMORY[2]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[0][2] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Third;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Third: {
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                HAL_Delay(1000);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            const char message[4] = {"KO!"};
            camera.Send(message,sizeof(message),100);
            HAL_Delay(50);
            robot.step = ActionState::Wait;
            break;
        }

        case ActionState::Wait:
            temp = 2 - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Finish;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        default:
            robot.step = ActionState::Finish;
            break;
    }
}

void Row2Event(){
    switch (robot.step) {
        case ActionState::Start://x1220y1990
            print("%s=%d \xff\xff\xff",HMI_valcontrol,50);
            robot.step = ActionState::Go;
            break;

        case ActionState::Go:
            MCRSBPtr->RunTaskTime(MoveDirection::Backward,970.0f);//x250y1990
            MCRSBPtr->RunTaskTime(MoveDirection::Rotate,88.0f);//x250y1900旋转
            MCRSBPtr->RunTaskTime(MoveDirection::Backward,915.0f);//x250y1075
            robot.step = ActionState::GoFirst;
            break;

        case ActionState::GoFirst:
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(100);
            robot.step = ActionState::First;
            break;

        case ActionState::First:
            camera.ReceiveDMA((uint8_t *)camera_message,14);
            if (camera_message[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
            };
            HAL_Delay(20);
            sscanf(camera_message,"#%d,%d,%d",camera_data+0,camera_data+1,camera_data+2);
            if (camera_data[0] == robot.mission.batchColor[1][0]) {
                Elevation_Move(21.45,Down);
                while (PULSE_COMPLETED != true){};
                HAL_Delay(100);
                arm.SetCompare(ARM_GRAB);
                HAL_Delay(1000);
                Elevation_Move(14.3,Up);
                while (PULSE_COMPLETED != true){};
                gimbal.SetCompare(GIMBAL_PLACE);
                HAL_Delay(2000);
                arm.SetCompare(ARM_PLACE);
                Elevation_Move(7.15,Up);
                while (PULSE_COMPLETED != true){};
                HAL_Delay(100);
                robot.step = ActionState::GoSecond;
            }
            break;

        case ActionState::GoSecond:
            camera_message[0] = 0;
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(50);
            plate.SetCompare(PLATE_MEMORY[1]);
            HAL_Delay(100);
            robot.step = ActionState::Second;
            break;

        case ActionState::Second:
            camera.ReceiveDMA((uint8_t *)camera_message,14);
            if (camera_message[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
            };
            HAL_Delay(20);
            sscanf(camera_message,"#%d,%d,%d",camera_data+0,camera_data+1,camera_data+2);
            if (camera_data[0] == robot.mission.batchColor[1][1]) {
                Elevation_Move(21.45,Down);
                while (PULSE_COMPLETED != true){};
                HAL_Delay(100);
                arm.SetCompare(ARM_GRAB);
                HAL_Delay(1000);
                Elevation_Move(14.3,Up);
                while (PULSE_COMPLETED != true){};
                gimbal.SetCompare(GIMBAL_PLACE);
                HAL_Delay(2000);
                arm.SetCompare(ARM_PLACE);
                Elevation_Move(7.15,Up);
                while (PULSE_COMPLETED != true){};
                HAL_Delay(100);
                robot.step = ActionState::GoThird;
            }
            break;

        case ActionState::GoThird:
            camera_message[0] = 0;
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(50);
            plate.SetCompare(PLATE_MEMORY[2]);
            HAL_Delay(100);
            robot.step = ActionState::Third;
            break;

        case ActionState::Third:
            camera.ReceiveDMA((uint8_t *)camera_message,14);
            while (camera_message[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
            };
            HAL_Delay(20);
            sscanf(camera_message,"#%d,%d,%d",camera_data+0,camera_data+1,camera_data+2);
            if (camera_data[0] == robot.mission.batchColor[1][2]) {
                Elevation_Move(21.45,Down);
                while (PULSE_COMPLETED != true){};
                HAL_Delay(100);
                arm.SetCompare(ARM_GRAB);
                HAL_Delay(1000);
                Elevation_Move(14.3,Up);
                while (PULSE_COMPLETED != true){};
                gimbal.SetCompare(GIMBAL_PLACE);
                HAL_Delay(2000);
                arm.SetCompare(ARM_PLACE);
                Elevation_Move(7.15,Up);
                while (PULSE_COMPLETED != true){};
                HAL_Delay(50);
                const char message[4] = {"OK!"};
                camera.Send(message,sizeof(message),100);
                robot.step = ActionState::Finish;
            }
            break;

        default:
            robot.step = ActionState::Finish;
            break;
        }
}

void Rough2Event(){
   switch (robot.step) {
        case ActionState::Start://x250y1090
           print("%s=%d \xff\xff\xff",HMI_valcontrol,62);
            camera_data[0] = 0;
            camera_data[1] = 0;
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(20);
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(20);
            int8_t temp;
            print("%s=%d \xff\xff\xff",HMI_valcontrol,11);
            robot.step = ActionState::Go;
            break;

        case ActionState::Go:
            MCRSBPtr->RunTaskTime(MoveDirection::Left,1835.0f);//x2085y1090
            MCRSBPtr->RunTaskTime(MoveDirection::Rotate,175.0f);//x2085y1090旋转
            robot.step = ActionState::GoFirst;
            break;

        case ActionState::GoFirst:
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[1][0] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
                }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y940
                Ring_Rough = Ring_Rough + temp;
                }
            else if (temp == 0) {
                robot.step = ActionState::First;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1240
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::First:
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(100);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
           if (camera_data[0] <= 0) {
               camera_data[0] = std::abs(camera_data[0]);
               MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
           }
           else if (camera_data[0] >= 0) {
               MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
           }
           if (camera_data[1] <= 0) {
               camera_data[1] = std::abs(camera_data[1]);
               MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
           }
           else if (camera_data[1] >= 0) {
               MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
           }
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::GoSecond;
            break;

        case ActionState::GoSecond:
            plate.SetCompare(PLATE_MEMORY[1]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[1][1] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Second;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Second:
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(100);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
           if (camera_data[0] <= 0) {
               camera_data[0] = std::abs(camera_data[0]);
               MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
           }
           else if (camera_data[0] >= 0) {
               MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
           }
           if (camera_data[1] <= 0) {
               camera_data[1] = std::abs(camera_data[1]);
               MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
           }
           else if (camera_data[1] >= 0) {
               MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
           }
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::GoThird;
            break;

        case ActionState::GoThird:
            plate.SetCompare(PLATE_MEMORY[2]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[1][2] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Third;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Third:
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(100);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
           if (camera_data[0] <= 0) {
               camera_data[0] = std::abs(camera_data[0]);
               MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
           }
           else if (camera_data[0] >= 0) {
               MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
           }
           if (camera_data[1] <= 0) {
               camera_data[1] = std::abs(camera_data[1]);
               MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
           }
           else if (camera_data[1] >= 0) {
               MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
           }
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(50);
            robot.step = ActionState::Wait;
            break;

        case ActionState::Wait:
            temp = 2 - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Finish;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        default:
            robot.step = ActionState::Finish;
            break;
    }
}

void Replace2Event(){
    switch (robot.step) {
        case ActionState::Start:////x2085y1070
            print("%s=%d \xff\xff\xff",HMI_valcontrol,75);
            robot.step = ActionState::GoFirst;
            break;

        case ActionState::GoFirst:
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(20);
            int8_t temp;
            temp = robot.mission.roughSlot[1][0] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::First;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::First:
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(100);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(20);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1000);
            Elevation_Move(7.15,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            Elevation_Move(7.15,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            robot.step = ActionState::GoSecond;
            break;

        case ActionState::GoSecond:
            plate.SetCompare(PLATE_MEMORY[1]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[1][1] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Second;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Second:
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(100);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(20);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1000);
            Elevation_Move(7.15,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            Elevation_Move(7.15,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            robot.step = ActionState::GoThird;
            break;

        case ActionState::GoThird:
            plate.SetCompare(PLATE_MEMORY[2]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[1][2] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Third;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;


        case ActionState::Third:
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(100);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(42.9,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(20);
            Elevation_Move(42.9,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1000);
            Elevation_Move(7.15,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            Elevation_Move(7.15,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(20);
            robot.step = ActionState::Wait;
            break;

        case ActionState::Wait:
            temp = 2 - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Finish;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        default:
            robot.step = ActionState::Finish;
            break;
    }
}

void Buffer2Event(){
    switch (robot.step) {
        case ActionState::Start://x2085y1070
            print("%s=%d \xff\xff\xff",HMI_valcontrol,87);
            MCRSBPtr->RunTaskTime(MoveDirection::Backward,830.0f);//x2085y1900
            MCRSBPtr->RunTaskTime(MoveDirection::Left,1015.0f);//x1070y1900
            int8_t temp;
            robot.step = ActionState::Go;
            break;

        case ActionState::Go:
            MCRSBPtr->RunTaskTime(MoveDirection::Rotate,88.0f);//x1070y1900
            robot.step = ActionState::GoFirst;
            break;

        case ActionState::GoFirst:
            plate.SetCompare(PLATE_MEMORY[0]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[1][0] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y940
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::First;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1240
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::First:
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(20.02,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                HAL_Delay(1000);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(20.02,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(20.02,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(20.02,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::GoSecond;
            break;

        case ActionState::GoSecond:
            plate.SetCompare(PLATE_MEMORY[1]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[1][1] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Second;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Second:
            Elevation_Move(20.02,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                HAL_Delay(1000);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(20.02,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(20.02,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(20.02,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::GoThird;
            break;

        case ActionState::GoThird:
            plate.SetCompare(PLATE_MEMORY[2]);
            HAL_Delay(20);
            temp = robot.mission.roughSlot[1][2] - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Third;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        case ActionState::Third:
            Elevation_Move(20.02,Down);
            while (PULSE_COMPLETED != true){};
            camera_data[0] = 0;
            camera_data[1] = 0;
            while (camera_data[0] == 0) {
                HAL_Delay(1000);
                camera.ReceiveDMA((uint8_t *)camera_message,14);
                HAL_Delay(20);
                sscanf(camera_message,"#%d,%d",camera_data+0,camera_data+1);
            };
            if (camera_data[0] <= 0) {
                camera_data[0] = std::abs(camera_data[0]);
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,camera_data[0] / 3.4);
            }
            else if (camera_data[0] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,camera_data[0] / 3.4);
            }
            if (camera_data[1] <= 0) {
                camera_data[1] = std::abs(camera_data[1]);
                MCRSBPtr->RunTaskTime(MoveDirection::Right,camera_data[1] / 1.4 + 3.0);
            }
            else if (camera_data[1] >= 0) {
                MCRSBPtr->RunTaskTime(MoveDirection::Left,camera_data[1] / 1.4 - 3.0);
            }
            Elevation_Move(20.02,Up);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(2000);
            gimbal.SetCompare(GIMBAL_PLACE);
            HAL_Delay(1500);
            Elevation_Move(14.3,Down);
            while (PULSE_COMPLETED != true){};
            arm.SetCompare(ARM_GRAB);
            HAL_Delay(1500);
            Elevation_Move(14.3,Up);
            while (PULSE_COMPLETED != true){};
            gimbal.SetCompare(GIMBAL_GRAB);
            HAL_Delay(1500);
            Elevation_Move(20.02,Down);
            while (PULSE_COMPLETED != true){};
            HAL_Delay(20);
            arm.SetCompare(ARM_PLACE);
            HAL_Delay(500);
            Elevation_Move(20.02,Up);
            while (PULSE_COMPLETED != true){};
            robot.step = ActionState::Wait;
            break;

        case ActionState::Wait:
            print("%s=%d \xff\xff\xff",HMI_valcontrol,100);
            temp = 2 - Ring_Rough;
            if (temp == -2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == -1){
                MCRSBPtr->RunTaskTime(MoveDirection::Forward,150.0f); // x2085y920
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 0) {
                robot.step = ActionState::Finish;
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 1) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,150.0f); // x2085y1220
                Ring_Rough = Ring_Rough + temp;
            }
            else if (temp == 2) {
                MCRSBPtr->RunTaskTime(MoveDirection::Backward,300.0f);
                Ring_Rough = Ring_Rough + temp;
            }
            break;

        default:
            robot.step = ActionState::Finish;
            break;
    }
}

void print(const char*format,...){
    char buf[512];
    va_list args;
    va_start(args,format);
    vsnprintf(buf,sizeof(buf),format,args);
    va_end(args);

    HMI.Send((uint8_t*)buf,strlen(buf) ,300);

    while(HAL_UART_GetState(&huart5)==HAL_UART_STATE_BUSY_TX);
}

void Elevation_Move(double dis,Dir dir){
    if (dir == Up) {
        dir_elevation.Write(GPIO_PIN_RESET);//UP
    }
    else if (dir == Down) {
        dir_elevation.Write(GPIO_PIN_SET);//Down
    }

    HAL_Delay(20);

    HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);

    double steps =  dis / 14.3 * 200;

    PULSE_COUNT  = 0;
    PULSE_TARGET = static_cast<uint32_t>(steps);
    PULSE_COMPLETED = false;

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
}

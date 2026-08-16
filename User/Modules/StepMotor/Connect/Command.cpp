//
// Created by Administrator on 25-2-28.
//

#include "Command.hpp"

//@brief: Set the current position as zero point

namespace Modules {

    using Command = StepMotorCH::Command;

    Command StepMotorCH::ResetZeroPoint() const {
        Command cmd;
        cmd[0] =  addr;                       // 地址
        cmd[1] =  0x0A;                       // 功能码
        cmd[2] =  0x6D;                       // 辅助码
        cmd[3] =  0x6B;
        cmd.size = 4;
        return cmd;
    }

    Command StepMotorCH::SetEnable(const bool _enable) const {
        Command cmd;

        cmd[0] =  addr;                       // 地址
        cmd[1] =  0xF3;                       // 功能码
        cmd[2] =  0xAB;                       // 辅助码
        cmd[3] =  static_cast<uint8_t>(_enable); // 使能状态
        cmd[4] =  0;                        // 多机同步运动标志
        cmd[5] =  0x6B;                       // 校验字节
        cmd.size  = 6;
        return cmd;
    }

    //@brief: Set the velocity of the motor
    //@param: _dir: the direction of the motor
    //@param: _corp: the flag of the motor
    //@param: _acc: the acceleration of the motor
    //@param: _velocity: the target velocity of the motor
    Command StepMotorCH::SetVelocity(const bool _dir, const bool _corp, const uint8_t _acc , const uint16_t _velocity) const {
        Command cmd;
        // 装载命令
        cmd[0] =  addr;                       // 地址
        cmd[1] =  0xF6;                       // 功能码
        cmd[2] =  _dir;                        // 方向
        cmd[3] =  static_cast<uint8_t>(_velocity >> 8);        // 速度(RPM)高8位字节
        cmd[4] =  static_cast<uint8_t>(_velocity >> 0);        // 速度(RPM)低8位字节
        cmd[5] =  _acc;                        // 加速度，注意：0是直接启动
        cmd[6] =  _corp;                        // 多机同步运动标志
        cmd[7] =  0x6B;                       // 校验字节
        cmd.size = 8;

        return cmd;
    }

    Command StepMotorCH::SetPosition(const bool _dir, const bool _corp, const uint8_t _acc, const uint16_t _velocity, const uint32_t _distance,
        const bool _relative) const {
        Command cmd;

        cmd[0]  =  addr;                      // 地址
        cmd[1]  =  0xFD;                      // 功能码
        cmd[2]  =  _dir;                       // 方向
        cmd[3]  =  static_cast<uint8_t>(_velocity >> 8);       // 速度(RPM)高8位字节
        cmd[4]  =  static_cast<uint8_t>(_velocity >> 0);       // 速度(RPM)低8位字节
        cmd[5]  =  _acc;                       // 加速度，注意：0是直接启动
        cmd[6]  =  static_cast<uint8_t>(_distance >> 24);      // 脉冲数(bit24 - bit31)
        cmd[7]  =  static_cast<uint8_t>(_distance >> 16);      // 脉冲数(bit16 - bit23)
        cmd[8]  =  static_cast<uint8_t>(_distance >> 8);       // 脉冲数(bit8  - bit15)
        cmd[9]  =  static_cast<uint8_t>(_distance >> 0);       // 脉冲数(bit0  - bit7 )
        cmd[10] =  _relative;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
        cmd[11] =  _corp;                       // 多机同步运动标志，false为不启用，true为启用
        cmd[12] =  0x6B;                      // 校验字节
        cmd.size = 13;

        return cmd;
    }

    Command StepMotorCH::StopNow() const {

        Command cmd;
        // 装载命令
        cmd[0] =  addr;                       // 地址
        cmd[1] =  0xFE;                       // 功能码
        cmd[2] =  0x98;                       // 辅助码
        cmd[3] =  false;                        // 多机同步运动标志
        cmd[4] =  0x6B;                       // 校验字节
        cmd.size = 5;

        return cmd;
    }

    Command StepMotorCH::RunSyncTask(const uint8_t _addr) {

        Command cmd;
        // 装载命令
        cmd[0] =  _addr;                       // 地址
        cmd[1] =  0xFF;                       // 功能码
        cmd[2] =  0x66;                       // 辅助码
        cmd[3] =  0x6B;                       // 校验字节
        cmd.size = 4;

        return cmd;

    }

    Command StepMotorCH::SetSubDivision(const uint8_t _sub) const {
        Command cmd;

        cmd[0] = addr;
        cmd[1] = 0x84;
        cmd[2] = 0x8A;
        cmd[3] = 0x01;
        cmd[4] = _sub;
        cmd[5] = 0x6B;

        cmd.size = 6;

        return cmd;
    }

    Command StepMotorCH::Calibration() const {

        Command cmd;

        cmd[0] = addr;
        cmd[1] = 0x06;
        cmd[2] = 0x45;
        cmd[3] = 0x6B;

        cmd.size = 4;

        return cmd;

    }

    Command StepMotorCH::ClearAngle() const {
        Command cmd;

        cmd[0] = addr;
        cmd[1] = 0x0A;
        cmd[2] = 0x6D;
        cmd[3] = 0x6B;

        cmd.size = 4;

        return cmd;
    }
}

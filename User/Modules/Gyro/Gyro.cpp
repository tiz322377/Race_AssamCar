//
// Created by Administrator on 24-9-4.
//
#ifdef HAL_I2C_MODULE_ENABLE
#include "Gyro.hpp"

namespace Modules {

    void Gyro<MPU6050Double>::update() {
        uint8_t data[14]{};
        port->MemRead(Addr1, 0x3B, 1, data, 14, 1000);
        Data.AccelX = (int16_t)(data[0] << 8 | data[1]);
        Data.AccelY = (int16_t)(data[2] << 8 | data[3]);
        Data.AccelZ = (int16_t)(data[4] << 8 | data[5]);
        Data.GyroX = (int16_t)(data[8] << 8 | data[9]);
        Data.GyroY = (int16_t)(data[10] << 8 | data[11]);
        Data.GyroZ = (int16_t)(data[12] << 8 | data[13]);

        for (unsigned char & i : data) {
            i = 0;
        }

        port->MemRead(Addr2, 0x3B, 1, data, 14, 1000);
        Data.AccelX += (int16_t)(data[0] << 8 | data[1]);
        Data.AccelY += (int16_t)(data[2] << 8 | data[3]);
        Data.AccelZ += (int16_t)(data[4] << 8 | data[5]);
        Data.GyroX += (int16_t)(data[8] << 8 | data[9]);
        Data.GyroY += (int16_t)(data[10] << 8 | data[11]);
        Data.GyroZ += (int16_t)(data[12] << 8 | data[13]);

        Data.AccelX /= 2;
        Data.AccelY /= 2;
        Data.AccelZ /= 2;
        Data.GyroX /= 2;
        Data.GyroY /= 2;
        Data.GyroZ /= 2;

        Data.AccelX -= Zero.AccelX;
        Data.AccelY -= Zero.AccelY;
        Data.AccelZ -= Zero.AccelZ;
        Data.GyroX -= Zero.GyroX;
        Data.GyroY -= Zero.GyroY;
        Data.GyroZ -= Zero.GyroZ;

    }

    void Gyro<MPU6050>::update() {
        uint8_t data[14]{};
        port->MemRead(Addr1, 0x3B, 1, data, 14, 1000);
        Data.AccelX = (int16_t)(data[0] << 8 | data[1]);
        Data.AccelY = (int16_t)(data[2] << 8 | data[3]);
        Data.AccelZ = (int16_t)(data[4] << 8 | data[5]);
        Data.GyroX = (int16_t)(data[8] << 8 | data[9]);
        Data.GyroY = (int16_t)(data[10] << 8 | data[11]);
        Data.GyroZ = (int16_t)(data[12] << 8 | data[13]);

        for (unsigned char & i : data) {
            i = 0;
        }

        Data.AccelX -= Zero.AccelX;
        Data.AccelY -= Zero.AccelY;
        Data.AccelZ -= Zero.AccelZ;
        Data.GyroX -= Zero.GyroX;
        Data.GyroY -= Zero.GyroY;
        Data.GyroZ -= Zero.GyroZ;

    }

} // Modules
#endif
//
// Created by Administrator on 24-9-4.
//

#ifndef TEST_Gyro_HPP
#define TEST_Gyro_HPP
#ifdef HAL_I2C_MODULE_ENABLE
#include "Peripheral/I2C.hpp"

namespace Modules {

    struct AccelData {
        int32_t AccelX = 0;
        int32_t AccelY = 0;
        int32_t AccelZ = 0;
        int32_t GyroX = 0;
        int32_t GyroY = 0;
        int32_t GyroZ = 0;
    };

    enum GyroType {
        MPU6050,
        MPU6050Double,
        Count = 100,
    };

    enum AccelAccuracy {
        ALow = 0x18, //16g
        AMedium = 0x10, //8g
        AHigh = 0x08, //4g
        AVeryHigh = 0x00, //2g
    };

    enum GyroAccuracy {
        GLow = 0x18, //2000
        GMedium = 0x10, //1000
        GHigh = 0x08, //500
        GVeryHigh = 0x00, //250
    };

    enum MPU6050Data {
        Addr1 = 0x68 << 1,
        Addr2 = 0x69 << 1,
        MPU6050DeceiveID = 0x75
    };

    template <GyroType> class Gyro;

    template<>
    class Gyro<MPU6050> {
        Peripheral::I2C<Peripheral::Master,Peripheral::I2CWorkMode::Normal> * port;

        AccelData Zero{};
        AccelData Data{};

        void update();

        void SetZero() {
            uint8_t data1[14 * Count]{};
            for (int i = 0; i < Count;i++) {
                port->MemRead(Addr1, 0x3B, 1, data1 + 14 * i, 14, 1000);
            }

            for (int i = 0; i < Count ;i++) {

                Zero.AccelX += static_cast<int16_t>((data1[14 * i] << 8 | data1[14 * i + 1]));
                Zero.AccelY += static_cast<int16_t>((data1[14 * i + 2] << 8 | data1[14 * i + 3]));
                Zero.AccelZ += static_cast<int16_t>((data1[14 * i + 4] << 8 | data1[14 * i + 5]));
                Zero.GyroX += static_cast<int16_t>((data1[14 * i + 8] << 8 | data1[14 * i + 9]));
                Zero.GyroY += static_cast<int16_t>((data1[14 * i + 10] << 8 | data1[14 * i + 11]));
                Zero.GyroZ += static_cast<int16_t>(data1[14 * i + 12] << 8 | data1[14 * i + 13]);

            }

            Zero.AccelX /= Count;
            Zero.AccelY /= Count;
            Zero.AccelZ /= Count;
            Zero.GyroX /= Count;
            Zero.GyroY /= Count;
            Zero.GyroZ /= Count;

        }

        void init(AccelAccuracy _accel,GyroAccuracy _Gyro) {
            uint8_t checkOne;
            uint8_t checkTwo;
            uint8_t data;

            // 检查MPU6050是否可用
            port->MemRead(Addr1, 0x75, 1, &checkOne, 1, 1000);
            if (checkOne == 0x68) {
                // 检查设备ID是否正确
                // 唤醒MPU6050
                data = 0x00;
                port->MemWrite(Addr1, 0x6B, 1, &data, 1);

                // 设置加速度计范围
                data = _accel;
                port->MemWrite(Addr1, 0x1C, 1, &data, 1, 1000);

                // 设置陀螺仪范围 ±250°/s
                data = _Gyro;
                port->MemWrite(Addr1, 0x1B, 1, &data, 1, 1000);

                // 设置采样率 = 1kHz / 1 = 1kHz
                data = 0x07;
                port->MemWrite(Addr1, 0x19, 1, &data, 1, 1000);

                // 配置数字低通滤波器（DLPF）为260Hz
                data = 0x00;
                port->MemWrite(Addr1, 0x1A, 1, &data, 1, 1000);
            }

            SetZero();
        }
    public:
        explicit Gyro(I2C_HandleTypeDef * hi2c, AccelAccuracy _accel = AVeryHigh, GyroAccuracy _Gyro = GVeryHigh) : port(new Peripheral::I2C<Peripheral::Master,Peripheral::I2CWorkMode::Normal>(hi2c)) {
            init(_accel,_Gyro);
        }
        const AccelData & GetData() {
            update();
            return Data;
        };

    };

    template<>
    class Gyro<MPU6050Double> {
        Peripheral::I2C<Peripheral::Master,Peripheral::I2CWorkMode::Normal> * port;

        AccelData Zero{};
        AccelData Data{};

        void update();

        void SetZero() {

            uint8_t data1[14 * Count]{};
            uint8_t data2[14 * Count]{};

            for (int i = 0; i < Count;i++) {
                port->MemRead(Addr1, 0x3B,1, data1 + 14 * i, 1, 100);
                port->MemRead( Addr2, 0x3B,1, data1 + 14 * i, 1, 100);
            }

            for (int i = 0; i < Count ;i++) {

                Zero.AccelX += static_cast<int16_t>((data1[14 * i] << 8 | data1[14 * i + 1]));
                Zero.AccelY += static_cast<int16_t>((data1[14 * i + 2] << 8 | data1[14 * i + 3]));
                Zero.AccelZ += static_cast<int16_t>((data1[14 * i + 4] << 8 | data1[14 * i + 5]));
                Zero.GyroX += static_cast<int16_t>((data1[14 * i + 8] << 8 | data1[14 * i + 9]));
                Zero.GyroY += static_cast<int16_t>((data1[14 * i + 10] << 8 | data1[14 * i + 11]));
                Zero.GyroZ += static_cast<int16_t>(data1[14 * i + 12] << 8 | data1[14 * i + 13]);

                Zero.AccelX += static_cast<int16_t>(((data2[14 * i] << 8 | data2[14 * i + 1])));
                Zero.AccelY += static_cast<int16_t>((data2[14 * i + 2] << 8 | data2[14 * i + 3]));
                Zero.AccelZ += static_cast<int16_t>((data2[14 * i + 4] << 8 | data2[14 * i + 5]));
                Zero.GyroX += static_cast<int16_t>((data2[14 * i + 8] << 8 | data2[14 * i + 9]));
                Zero.GyroY += static_cast<int16_t>((data2[14 * i + 10] << 8 | data2[14 * i + 11]));
                Zero.GyroZ += static_cast<int16_t>((data2[14 * i + 12] << 8 | data2[14 * i + 13]));

            }

            Zero.AccelX /= 2 * Count;
            Zero.AccelY /= 2 * Count;
            Zero.AccelZ /= 2 * Count;
            Zero.GyroX /=  2 * Count;
            Zero.GyroY /=  2 * Count;
            Zero.GyroZ /=  2 * Count;

        }

    public:
        explicit Gyro(Peripheral::I2C<Peripheral::Master,Peripheral::I2CWorkMode::Normal> * i2c) : port(i2c) {
            Init(AVeryHigh, GVeryHigh);
        }

        void Init(AccelAccuracy _accel,GyroAccuracy _Gyro) {
            uint8_t checkOne = 0;
            uint8_t checkTwo = 0;
            uint8_t data = 0;

            // 检查MPU6050是否可用
            while(checkOne != 0x68 && checkTwo != 0x68) {
                port->MemRead(Addr2, MPU6050DeceiveID, 1, &checkTwo, 1, 1000);
                port->MemRead(Addr1, MPU6050DeceiveID, 1, &checkOne, 1, 1000);
            }
            if (checkOne == 0x68) {
                // 检查设备ID是否正确
                // 唤醒MPU6050
                data = 0x00;
                port->MemWrite(Addr1, 0x75,1, &data, 1, 100);
                port->MemWrite(Addr2, 0x75,1, &data, 1, 100);

                // 设置加速度计范围
                data = _accel;
                port->MemWrite(Addr1, 0x75,1, &data, 1, 100);
                port->MemWrite( Addr2, 0x75,1, &data, 1, 100);

                // 设置陀螺仪范围 ±250°/s
                data = _Gyro;
                port->MemWrite(Addr1, 0x75,1, &data, 1, 100);
                port->MemWrite(Addr2, 0x75,1, &data, 1, 100);

                // 设置采样率 = 1kHz / 1 = 1kHz
                data = 0x07;
                port->MemWrite(Addr1, 0x75,1, &data, 1, 100);
                port->MemWrite(Addr2, 0x75,1, &data, 1, 100);

                // 配置数字低通滤波器（DLPF）为260Hz
                data = 0x00;
                port->MemWrite(Addr1, 0x75,1, &data, 1, 100);
                port->MemWrite(Addr2, 0x75,1, &data, 1, 100);
            }

            SetZero();
        }
        const AccelData & GetData() {
            update();
            return Data;
        }
    };

} // Modules
#endif
#endif //TEST_Gyro_HPP

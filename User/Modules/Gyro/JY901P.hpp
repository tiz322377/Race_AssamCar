#ifndef JY901P_HPP
#define JY901P_HPP

#include "Config/Config.hpp"

#ifdef HAL_I2C_MODULE_ENABLED

#include "Peripheral/I2C.hpp"

#include <cstdint>

namespace Modules {

    struct JY901PVector3 {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct JY901PRawData {
        int16_t accelerationX = 0;
        int16_t accelerationY = 0;
        int16_t accelerationZ = 0;
        int16_t angularVelocityX = 0;
        int16_t angularVelocityY = 0;
        int16_t angularVelocityZ = 0;
        int16_t magneticFieldX = 0;
        int16_t magneticFieldY = 0;
        int16_t magneticFieldZ = 0;
        int16_t roll = 0;
        int16_t pitch = 0;
        int16_t yaw = 0;
        int16_t temperature = 0;
    };

    struct JY901PData {
        JY901PVector3 accelerationG{};
        JY901PVector3 angularVelocityDegreesPerSecond{};
        JY901PVector3 magneticFieldLsb{};
        JY901PVector3 angleDegrees{};
        float temperatureC = 0.0f;
    };

    struct JY901PQuaternion {
        float q0 = 0.0f;
        float q1 = 0.0f;
        float q2 = 0.0f;
        float q3 = 0.0f;
    };

    class JY901P {
    public:
        static constexpr uint8_t defaultAddress = 0x50;

        enum class Register : uint8_t {
            Save              = 0x00,
            Calibration       = 0x01,
            OutputContent     = 0x02,
            OutputRate        = 0x03,
            BaudRate          = 0x04,
            I2CAddress        = 0x1A,
            LedOff            = 0x1B,
            Bandwidth         = 0x1F,
            Sleep             = 0x22,
            Orientation       = 0x23,
            Algorithm         = 0x24,
            FusionFilter      = 0x25,
            AccelerometerFilter = 0x2A,
            Version           = 0x2E,
            AccelerationX     = 0x34,
            AccelerationY     = 0x35,
            AccelerationZ     = 0x36,
            AngularVelocityX  = 0x37,
            AngularVelocityY  = 0x38,
            AngularVelocityZ  = 0x39,
            MagneticFieldX    = 0x3A,
            MagneticFieldY    = 0x3B,
            MagneticFieldZ    = 0x3C,
            Roll              = 0x3D,
            Pitch             = 0x3E,
            Yaw               = 0x3F,
            Temperature       = 0x40,
            Quaternion0       = 0x51,
            Quaternion1       = 0x52,
            Quaternion2       = 0x53,
            Quaternion3       = 0x54,
            Key               = 0x69
        };

        enum class CalibrationMode : uint16_t {
            Normal               = 0x00,
            Accelerometer        = 0x01,
            HeightZero           = 0x03,
            HeadingZero          = 0x04,
            MagneticSphere       = 0x07,
            AngleReference       = 0x08,
            MagneticDoublePlane  = 0x09
        };

        enum class OutputRate : uint16_t {
            Hz0_2  = 0x01,
            Hz0_5  = 0x02,
            Hz1    = 0x03,
            Hz2    = 0x04,
            Hz5    = 0x05,
            Hz10   = 0x06,
            Hz20   = 0x07,
            Hz50   = 0x08,
            Hz100  = 0x09,
            Hz200  = 0x0B,
            Single = 0x0C
        };

        enum class Bandwidth : uint16_t {
            Hz256 = 0x00,
            Hz188 = 0x01,
            Hz98  = 0x02,
            Hz42  = 0x03,
            Hz20  = 0x04,
            Hz10  = 0x05,
            Hz5   = 0x06
        };

        enum class InstallationDirection : uint16_t {
            Horizontal = 0x00,
            Vertical   = 0x01
        };

        enum class Algorithm : uint16_t {
            Axis9 = 0x00,
            Axis6 = 0x01
        };

        explicit JY901P(I2C_HandleTypeDef *_handle, uint8_t _address = defaultAddress, uint32_t _timeoutMs = 100);

        JY901P(const JY901P &)            = delete;
        JY901P &operator=(const JY901P &) = delete;

        [[nodiscard]] bool IsReady(uint32_t _trials = 2) const;
        [[nodiscard]] bool ReadRegister(Register _register, uint16_t &_value) const;
        [[nodiscard]] bool ReadRegisters(Register _startRegister, uint16_t *_values, uint8_t _count) const;
        [[nodiscard]] bool WriteRegister(Register _register, uint16_t _value) const;

        [[nodiscard]] bool ReadRawData(JY901PRawData &_data) const;
        [[nodiscard]] bool ReadData(JY901PData &_data) const;
        [[nodiscard]] bool ReadAngles(JY901PVector3 &_anglesDegrees) const;
        [[nodiscard]] bool ReadYaw(float &_yawDegrees) const;
        [[nodiscard]] bool ReadQuaternion(JY901PQuaternion &_quaternion) const;
        [[nodiscard]] bool ReadVersion(uint16_t &_version) const;

        [[nodiscard]] bool Unlock() const;
        [[nodiscard]] bool SaveConfiguration() const;
        [[nodiscard]] bool Restart() const;
        [[nodiscard]] bool RestoreFactorySettings() const;
        [[nodiscard]] bool SetCalibrationMode(CalibrationMode _mode) const;
        [[nodiscard]] bool SetOutputRate(OutputRate _rate) const;
        [[nodiscard]] bool SetBandwidth(Bandwidth _bandwidth) const;
        [[nodiscard]] bool SetInstallationDirection(InstallationDirection _direction) const;
        [[nodiscard]] bool SetAlgorithm(Algorithm _algorithm) const;
        [[nodiscard]] bool SetLedEnabled(bool _enabled) const;
        [[nodiscard]] bool SetFusionFilter(uint16_t _value) const;
        [[nodiscard]] bool SetAccelerometerFilter(uint16_t _value) const;
        [[nodiscard]] bool SetAddress(uint8_t _address);

        [[nodiscard]] uint8_t GetAddress() const;

    private:
        using I2CBus = Peripheral::I2C<Peripheral::Master, Peripheral::I2CWorkMode::Normal>;

        static constexpr float accelerationScale = 16.0f / 32768.0f;
        static constexpr float angularVelocityScale = 2000.0f / 32768.0f;
        static constexpr float angleScale = 180.0f / 32768.0f;
        static constexpr float temperatureScale = 1.0f / 100.0f;
        static constexpr float quaternionScale = 1.0f / 32768.0f;

        [[nodiscard]] static bool isAddressValid(uint8_t _address);
        [[nodiscard]] static int16_t decodeSigned(const uint8_t *_data);
        [[nodiscard]] uint16_t deviceAddress() const;
        [[nodiscard]] bool readBytes(Register _register, uint8_t *_data, uint16_t _size) const;

        I2CBus bus;
        uint8_t address;
        const uint32_t timeoutMs;
    };

} // namespace Modules

#endif // HAL_I2C_MODULE_ENABLED

#endif // JY901P_HPP

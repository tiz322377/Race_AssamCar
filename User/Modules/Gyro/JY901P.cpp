#include "Modules/Gyro/JY901P.hpp"

#ifdef HAL_I2C_MODULE_ENABLED

namespace Modules {

    JY901P::JY901P(I2C_HandleTypeDef *_handle, const uint8_t _address, const uint32_t _timeoutMs)
        : bus(_handle), address(_address), timeoutMs(_timeoutMs)
    {
    }

    bool JY901P::IsReady(const uint32_t _trials) const
    {
        if (!isAddressValid(address) || bus.Port == nullptr || _trials == 0U) {
            return false;
        }

        return HAL_I2C_IsDeviceReady(bus.Port, deviceAddress(), _trials, timeoutMs) == HAL_OK;
    }

    bool JY901P::ReadRegister(const Register _register, uint16_t &_value) const
    {
        uint8_t data[2]{};
        if (!readBytes(_register, data, sizeof(data))) {
            return false;
        }

        _value = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U);
        return true;
    }

    bool JY901P::ReadRegisters(const Register _startRegister, uint16_t *_values, const uint8_t _count) const
    {
        if (_values == nullptr || _count == 0U) {
            return false;
        }

        const auto startAddress = static_cast<uint8_t>(_startRegister);
        if (static_cast<uint16_t>(startAddress) + _count > 0x100U) {
            return false;
        }

        for (uint16_t i = 0; i < _count; ++i) {
            const auto currentRegister = static_cast<Register>(static_cast<uint8_t>(startAddress + i));
            if (!ReadRegister(currentRegister, _values[i])) {
                return false;
            }
        }

        return true;
    }

    bool JY901P::WriteRegister(const Register _register, const uint16_t _value) const
    {
        if (!isAddressValid(address) || bus.Port == nullptr) {
            return false;
        }

        uint8_t data[2]{
            static_cast<uint8_t>(_value & 0xFFU),
            static_cast<uint8_t>(_value >> 8U)
        };

        return bus.MemWrite(deviceAddress(), static_cast<uint8_t>(_register), I2C_MEMADD_SIZE_8BIT, data, sizeof(data), timeoutMs) == HAL_OK;
    }

    bool JY901P::ReadRawData(JY901PRawData &_data) const
    {
        uint8_t data[26]{};
        if (!readBytes(Register::AccelerationX, data, sizeof(data))) {
            return false;
        }

        _data.accelerationX    = decodeSigned(data + 0);
        _data.accelerationY    = decodeSigned(data + 2);
        _data.accelerationZ    = decodeSigned(data + 4);
        _data.angularVelocityX = decodeSigned(data + 6);
        _data.angularVelocityY = decodeSigned(data + 8);
        _data.angularVelocityZ = decodeSigned(data + 10);
        _data.magneticFieldX   = decodeSigned(data + 12);
        _data.magneticFieldY   = decodeSigned(data + 14);
        _data.magneticFieldZ   = decodeSigned(data + 16);
        _data.roll             = decodeSigned(data + 18);
        _data.pitch            = decodeSigned(data + 20);
        _data.yaw              = decodeSigned(data + 22);
        _data.temperature      = decodeSigned(data + 24);
        return true;
    }

    bool JY901P::ReadData(JY901PData &_data) const
    {
        JY901PRawData raw{};
        if (!ReadRawData(raw)) {
            return false;
        }

        _data.accelerationG = {
            static_cast<float>(raw.accelerationX) * accelerationScale,
            static_cast<float>(raw.accelerationY) * accelerationScale,
            static_cast<float>(raw.accelerationZ) * accelerationScale
        };
        _data.angularVelocityDegreesPerSecond = {
            static_cast<float>(raw.angularVelocityX) * angularVelocityScale,
            static_cast<float>(raw.angularVelocityY) * angularVelocityScale,
            static_cast<float>(raw.angularVelocityZ) * angularVelocityScale
        };
        _data.magneticFieldLsb = {
            static_cast<float>(raw.magneticFieldX),
            static_cast<float>(raw.magneticFieldY),
            static_cast<float>(raw.magneticFieldZ)
        };
        _data.angleDegrees = {
            static_cast<float>(raw.roll) * angleScale,
            static_cast<float>(raw.pitch) * angleScale,
            static_cast<float>(raw.yaw) * angleScale
        };
        _data.temperatureC = static_cast<float>(raw.temperature) * temperatureScale;
        return true;
    }

    bool JY901P::ReadAngles(JY901PVector3 &_anglesDegrees) const
    {
        uint8_t data[6]{};
        if (!readBytes(Register::Roll, data, sizeof(data))) {
            return false;
        }

        _anglesDegrees = {
            static_cast<float>(decodeSigned(data + 0)) * angleScale,
            static_cast<float>(decodeSigned(data + 2)) * angleScale,
            static_cast<float>(decodeSigned(data + 4)) * angleScale
        };
        return true;
    }

    bool JY901P::ReadQuaternion(JY901PQuaternion &_quaternion) const
    {
        uint8_t data[8]{};
        if (!readBytes(Register::Quaternion0, data, sizeof(data))) {
            return false;
        }

        _quaternion.q0 = static_cast<float>(decodeSigned(data + 0)) * quaternionScale;
        _quaternion.q1 = static_cast<float>(decodeSigned(data + 2)) * quaternionScale;
        _quaternion.q2 = static_cast<float>(decodeSigned(data + 4)) * quaternionScale;
        _quaternion.q3 = static_cast<float>(decodeSigned(data + 6)) * quaternionScale;
        return true;
    }

    bool JY901P::ReadVersion(uint16_t &_version) const
    {
        return ReadRegister(Register::Version, _version);
    }

    bool JY901P::Unlock() const
    {
        return WriteRegister(Register::Key, 0xB588U);
    }

    bool JY901P::SaveConfiguration() const
    {
        return WriteRegister(Register::Save, 0x0000U);
    }

    bool JY901P::Restart() const
    {
        return WriteRegister(Register::Save, 0x00FFU);
    }

    bool JY901P::RestoreFactorySettings() const
    {
        return WriteRegister(Register::Save, 0x0001U);
    }

    bool JY901P::SetCalibrationMode(const CalibrationMode _mode) const
    {
        return WriteRegister(Register::Calibration, static_cast<uint16_t>(_mode));
    }

    bool JY901P::SetOutputRate(const OutputRate _rate) const
    {
        return WriteRegister(Register::OutputRate, static_cast<uint16_t>(_rate));
    }

    bool JY901P::SetBandwidth(const Bandwidth _bandwidth) const
    {
        return WriteRegister(Register::Bandwidth, static_cast<uint16_t>(_bandwidth));
    }

    bool JY901P::SetInstallationDirection(const InstallationDirection _direction) const
    {
        return WriteRegister(Register::Orientation, static_cast<uint16_t>(_direction));
    }

    bool JY901P::SetAlgorithm(const Algorithm _algorithm) const
    {
        return WriteRegister(Register::Algorithm, static_cast<uint16_t>(_algorithm));
    }

    bool JY901P::SetLedEnabled(const bool _enabled) const
    {
        return WriteRegister(Register::LedOff, _enabled ? 0x0000U : 0x0001U);
    }

    bool JY901P::SetFusionFilter(const uint16_t _value) const
    {
        if (_value < 1U || _value > 10000U) {
            return false;
        }

        return WriteRegister(Register::FusionFilter, _value);
    }

    bool JY901P::SetAccelerometerFilter(const uint16_t _value) const
    {
        if (_value < 1U || _value > 10000U) {
            return false;
        }

        return WriteRegister(Register::AccelerometerFilter, _value);
    }

    bool JY901P::SetAddress(const uint8_t _address)
    {
        if (!isAddressValid(_address) || !WriteRegister(Register::I2CAddress, _address)) {
            return false;
        }

        address = _address;
        return true;
    }

    uint8_t JY901P::GetAddress() const
    {
        return address;
    }

    bool JY901P::isAddressValid(const uint8_t _address)
    {
        return _address >= 0x01U && _address <= 0x7FU;
    }

    int16_t JY901P::decodeSigned(const uint8_t *_data)
    {
        const auto value = static_cast<uint16_t>(_data[0]) | (static_cast<uint16_t>(_data[1]) << 8U);
        return static_cast<int16_t>(value);
    }

    uint16_t JY901P::deviceAddress() const
    {
        return static_cast<uint16_t>(address) << 1U;
    }

    bool JY901P::readBytes(const Register _register, uint8_t *_data, const uint16_t _size) const
    {
        if (!isAddressValid(address) || bus.Port == nullptr || _data == nullptr || _size == 0U) {
            return false;
        }

        return bus.MemRead(deviceAddress(), static_cast<uint8_t>(_register), I2C_MEMADD_SIZE_8BIT, _data, _size, timeoutMs) == HAL_OK;
    }

} // namespace Modules

#endif // HAL_I2C_MODULE_ENABLED

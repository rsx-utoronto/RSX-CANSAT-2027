#include "lsm6dso.hpp"
#include <stdint.h>
#include <Wire.h>

void Lsm6dso::i2c_write(uint8_t regAddr, uint8_t data)
{
    // Start-Transfer waveform
    this->_bus.beginTransmission(this->_address);
    // Choose Register
    this->_bus.write(regAddr);
    // Write Data
    this->_bus.write(data);
    // End Transfer
    this->_bus.endTransmission();
}

void Lsm6dso::i2c_read(uint8_t regAddr, uint8_t* buffer, int readSize)
{
    //Select Address
    this->_bus.beginTransmission(this->_address);
    this->_bus.write(regAddr);
    this->_bus.endTransmission(false);

    this->_bus.requestFrom(this->_address, readSize);
    for(int i = 0; i < readSize; i++)
    {
        buffer[i] = this->_bus.read();
    }
}

int16_t i2c_read16(Lsm6dso* obj, uint8_t addr_L, uint8_t addr_H) // Helper for reading 16-bit numbers
{
    int16_t value = 0;

    uint8_t lower = 0;
    uint8_t upper = 0;
    

    obj->i2c_read(addr_L, &lower, 1);
    obj->i2c_read(addr_H, &upper, 1);

    value = (upper << 8) | lower;

    return value;
}

float Lsm6dso::get_XpitchAngularVelocity()
{
    return i2c_read16(this, LSM6DSO_OUTX_L_G, LSM6DSO_OUTX_H_G) * GYRO_INT16_TO_FLOAT;
}

float Lsm6dso::get_YrollAngularVelocity()
{
    return i2c_read16(this, LSM6DSO_OUTY_L_G, LSM6DSO_OUTY_H_G) * GYRO_INT16_TO_FLOAT;
}

float Lsm6dso::get_ZyawAngularVelocity()
{
    return i2c_read16(this, LSM6DSO_OUTZ_L_G, LSM6DSO_OUTZ_H_G) * GYRO_INT16_TO_FLOAT;
}

gyroAngularVelocityData Lsm6dso::get_AngularVelocity()
{
    return {
        this->get_XpitchAngularVelocity(),
        this->get_YrollAngularVelocity(),
        this->get_ZyawAngularVelocity()
    };
}

bool Lsm6dso::begin(uint8_t addr,  uint8_t mode_ODR, TwoWire& bus)
{
    // Store data
    this->_address = addr;
    // Begin bus
    this->i2c_write(0x11, mode_ODR);

    return true;
}
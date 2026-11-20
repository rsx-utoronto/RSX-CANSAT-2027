#include "lsm6dso.hpp"
#include <stdint.h>
#include <Wire.h>

void Lsm6dso::i2c_write(uint8_t regAddr, uint16_t data)
{
    // Start-Transfer waveform
    this->_bus.beginTransmission(this->_address);
    // Choose Register
    this->_bus.write(regAddr);
    // Write Data

    // End Transfer
    this->_bus.endTransmission();
}

void Lsm6dso::i2c_read(uint8_t regAddr, uint8_t* buffer, int readSize)
{
    //Select Address
    this->_bus.beginTransmission(this->_address);
    this->_bus.write(regAddr);
    this->_bus.endTransmission();

    this->_bus.requestFrom(this->_address, readSize);
    for(int i = 0; i < readSize; i++)
    {
        buffer[i] = this->_bus.read();
    }
}

float Lsm6dso::get_XpitchAngularVelocity()
{
    int16_t value = 0;

    uint8_t lower = 0;
    uint8_t upper = 0;
    

    i2c_read(LSM6DSO_OUTX_L_G, &lower, 1);
    i2c_read(LSM6DSO_OUTX_H_G, &upper, 1);

    value = (upper << 8) | lower;

    return value * 0.00875f;
}

float Lsm6dso::get_YrollAngularVelocity()
{
    int16_t value = 0;

    uint8_t lower = 0;
    uint8_t upper = 0;
    

    i2c_read(LSM6DSO_OUTY_L_G, &lower, 1);
    i2c_read(LSM6DSO_OUTY_H_G, &upper, 1);

    value = (upper << 8) | lower;

    return value * 0.00875f;
}

float Lsm6dso::get_ZyawAngularVelocity()
{
    int16_t value = 0;

    uint8_t lower = 0;
    uint8_t upper = 0;
    

    i2c_read(LSM6DSO_OUTZ_L_G, &lower, 1);
    i2c_read(LSM6DSO_OUTZ_H_G, &upper, 1);

    value = (upper << 8) | lower;

    return value * 0.00875f;
}

gyroAngularVelocityData Lsm6dso::get_AngularVelocity()
{
    return {
        this->get_XpitchAngularVelocity(),
        this->get_YrollAngularVelocity(),
        this->get_ZyawAngularVelocity()
    };
}

bool Lsm6dso::begin(uint8_t addr, TwoWire& bus = Wire)
{
    // Store data
    this->_address = addr;
    this->_bus = bus;
    // Begin bus
    this->_bus.begin();
}
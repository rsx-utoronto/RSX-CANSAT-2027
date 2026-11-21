#pragma once
#include <stdint.h>
#include <Wire.h>


#define LSM6DSO_ADDR_SDO_HIGH 0b1101011 // Addr when SDO/SA0 pin connected to supply
#define LSM6DSO_ADDR_SDO_LOW 0b11010110 // Addr when SDO/SA0 pin connected to ground

// I2C Registers
#define LSM6DSO_OUT_TEMP_L              0x20    // Temperature sensor output data LSB (16-bit signed, 256 LSB/degC, 0 LSB at 25 degC)
#define LSM6DSO_OUT_TEMP_H              0x21    // Temperature sensor output data MSB



#define LSM6DSO_OUTX_L_G                0x22    // Gyroscope pitch/X-axis angular rate output LSB
#define LSM6DSO_OUTX_H_G                0x23    // Gyroscope pitch/X-axis angular rate output MSB

#define LSM6DSO_OUTY_L_G                0x24    // Gyroscope roll/Y-axis angular rate output LSB
#define LSM6DSO_OUTY_H_G                0x25    // Gyroscope roll/Y-axis angular rate output MSB

#define LSM6DSO_OUTZ_L_G                0x26    // Gyroscope yaw/Z-axis angular rate output LSB
#define LSM6DSO_OUTZ_H_G                0x27    // Gyroscope yaw/Z-axis angular rate output MSB



#define LSM6DSO_OUTX_L_A                0x28    // Accelerometer X-axis linear acceleration output LSB
#define LSM6DSO_OUTX_H_A                0x29    // Accelerometer X-axis linear acceleration output MSB

#define LSM6DSO_OUTY_L_A                0x2A    // Accelerometer Y-axis linear acceleration output LSB
#define LSM6DSO_OUTY_H_A                0x2B    // Accelerometer Y-axis linear acceleration output MSB

#define LSM6DSO_OUTZ_L_A                0x2C    // Accelerometer Z-axis linear acceleration output LSB
#define LSM6DSO_OUTZ_H_A                0x2D    // Accelerometer Z-axis linear acceleration output MSB



#define LSM6DSO_TIMESTAMP0              0x40    // 32-bit internal timestamp counter output Byte 0 (LSB, 25 us/LSB typical)
#define LSM6DSO_TIMESTAMP1              0x41    // 32-bit internal timestamp counter output Byte 1
#define LSM6DSO_TIMESTAMP2              0x42    // 32-bit internal timestamp counter output Byte 2
#define LSM6DSO_TIMESTAMP3              0x43    // 32-bit internal timestamp counter output Byte 3 (MSB)



#define LSM6DSO_FIFO_DATA_OUT_TAG       0x78    // FIFO 7-byte word header identifying sensor source (TAG_SENSOR[4:0]), counter, and parity



#define LSM6DSO_FIFO_DATA_OUT_X_L       0x79    // FIFO buffered X-axis (or first 16-bit word) data LSB
#define LSM6DSO_FIFO_DATA_OUT_X_H       0x7A    // FIFO buffered X-axis (or first 16-bit word) data MSB

#define LSM6DSO_FIFO_DATA_OUT_Y_L       0x7B    // FIFO buffered Y-axis (or second 16-bit word) data LSB
#define LSM6DSO_FIFO_DATA_OUT_Y_H       0x7C    // FIFO buffered Y-axis (or second 16-bit word) data MSB

#define LSM6DSO_FIFO_DATA_OUT_Z_L       0x7D    // FIFO buffered Z-axis (or third 16-bit word) data LSB
#define LSM6DSO_FIFO_DATA_OUT_Z_H       0x7E    // FIFO buffered Z-axis (or third 16-bit word) data MSB

#define ODR_HIGH_PERF                   0x50    // Config map key for high sampling-rate mode
#define ODR_NORMAL_PERF                 0x40    // Config map key for normal sampling-rate mode

const float GYRO_INT16_TO_FLOAT = 0.00875f;

struct gyroAngularVelocityData
{
    float x;    //pitch
    float y;    //roll
    float z;    //yaw
};

class Lsm6dso {
    public:
        // Setup sequence
        Lsm6dso(TwoWire& bus) : _bus(bus){};
        bool begin(uint8_t addr = LSM6DSO_ADDR_SDO_LOW, uint8_t mode_ODR = ODR_NORMAL_PERF, TwoWire& bus = Wire);
        void i2c_read(uint8_t regAddr, uint8_t* buffer, int readSize);  // Generic I2C read for object
        // Register Getters
        float get_XpitchAngularVelocity();              // Return gyro pitch
        float get_YrollAngularVelocity();               // Return gyro roll
        float get_ZyawAngularVelocity();                // Return gyro yaw
        gyroAngularVelocityData get_AngularVelocity();  // Return gyro angular velocity vector

    

    private:
        int _address = 0;                                   // I2C device address
        TwoWire& _bus;                                  // Bus object for I2C
        
        void i2c_write(uint8_t regAddr, uint8_t data);                 // Generic I2C write for object
        

        

        

};
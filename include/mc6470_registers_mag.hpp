#ifndef __MC6470_REGISTERS_MAG_HPP
#define __MC6470_REGISTERS_MAG_HPP

#include <Arduino.h>

namespace MC6470
{
    const uint8_t address_mag = 0x0c;

    const uint8_t mag_whoami_address = 0x0f;
    const uint8_t mag_whoami_expected = 0x49;
    const int error_mag_whoami_mismatch = -1;

    typedef enum : uint8_t
    {
        PC = 0b10000000,
        ODR_HZ0P5 = 0b00000000,
        ODR_HZ10 = 0b00001000,
        ODR_HZ20 = 0b00010000,
        ODR_HZ100 = 0b00011000,
        FS = 0b00000010,
    } MAG_CTRL1_REG;
    const uint8_t mag_ctrl1_reg_address = 0x1B;

    typedef enum : uint8_t
    {
        FORCE = 0b01000000,
        TCS = 0b00000100,
        OCL = 0b00001000
    } MAG_CTRL3_REG;
    const uint8_t mag_ctrl3_reg_address = 0x1D;

    typedef enum : uint8_t
    {
        DRDY = 0b01000000,
    } MAG_STATUS_REG;
    const uint8_t mag_status_reg_address = 0x18;

    typedef enum : uint8_t
    {
        RANGE_14BIT = 0b00000000,
        RANGE_15BIT = 0b00010000,
    } MAG_RANGE_REG;
    const uint8_t mag_range_reg_address = 0x1E;
}

#endif
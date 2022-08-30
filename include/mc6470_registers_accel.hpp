// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#ifndef INCLUDE_MC6470_REGISTERS_ACCEL_HPP_
#define INCLUDE_MC6470_REGISTERS_ACCEL_HPP_

#include <Arduino.h>
#include <map>
#include <utility>

namespace MC6470
{
    const uint8_t address_accel_a5gnd = 0x4c;
    const uint8_t address_accel_a5vcc = 0x6c;

    const uint8_t acc_xyz_reg_address = 0x0D;

    typedef enum : uint8_t
    {
        ACQ_INT = 0b10000000,
    } ACC_SR_REG;
    const uint8_t acc_sr_reg_address = 0x03;

    typedef enum : uint8_t
    {
        RATE_HZ32 = 0b00000000,
        RATE_HZ16 = 0b00000001,
        RATE_HZ8 = 0b00000010,
        RATE_HZ4 = 0b00000011,
        RATE_HZ2 = 0b00000100,
        RATE_HZ1 = 0b00000101,
        RATE_HZ0P5 = 0b00000110,
        RATE_HZ0P25 = 0b00000111,
        RATE_HZ64 = 0b00001000,
        RATE_HZ128 = 0b00001001,
        RATE_HZ256 = 0b00001010,
        RATE_MASK = 0b00001111,
    } ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG;
    const uint8_t acc_srtf_reg_address = 0x08;

    extern std::map<uint8_t, float> acc_get_rate_hz;

    typedef enum : uint8_t
    {
        OPCON_STANDBY = 0b00000000,
        OPCON_WAKE = 0b00000001,
        OPCON_MASK = 0b00000011
    } ACC_MODE_REG;
    const uint8_t acc_mode_reg_address = 0x07;

    typedef enum : uint8_t
    {
        ACQ_INT_EN = 0b10000000,
        // there are tap interrupts here too.
    } ACC_INTEN_REG;
    const uint8_t acc_inten_reg_address = 0x06;

    typedef enum : uint8_t
    {
        OPSTAT_STANDBY = 0b00000000,
        OPSTAT_WAKE = 0b00000001,
        OPSTAT_MASK = 0b00000011,

        OTPA_IDLE = 0b00000000,
        OTPA_ACTIVE = 0b10000000,
        OTPA_MASK = 0b10000000,
    } ACC_OPSTAT_REG;
    const uint8_t acc_opstat_reg_address = 0x04;

    typedef enum : uint8_t
    {
        RES_6BIT = 0b00000000,
        RES_7BIT = 0b00000001,
        RES_8BIT = 0b00000010,
        RES_10BIT = 0b00000011,
        RES_12BIT = 0b00000100,
        RES_14BIT = 0b00000101,
        RES_MASK = 0b00000111,

        RANGE_2G = 0b00000000,
        RANGE_4G = 0b00010000,
        RANGE_8G = 0b00100000,
        RANGE_16G = 0b00110000,
        RANGE_MASK = 0b11110000, // Hack: bit 7 must always be 0, so range now sets it to that
    } ACC_OUTCFG_REG;
    const uint8_t acc_outcfg_reg_address = 0x20;
} // namespace MC6470

#endif // INCLUDE_MC6470_REGISTERS_ACCEL_HPP_

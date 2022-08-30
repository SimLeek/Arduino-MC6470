// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470_registers_mag.hpp"

namespace MC6470
{
std::map<uint8_t, float> mag_get_rate_hz{
        std::make_pair(MAG_CTRL1_REG::ODR_HZ0P5, 0.5),
        std::make_pair(MAG_CTRL1_REG::ODR_HZ10, 10),
        std::make_pair(MAG_CTRL1_REG::ODR_HZ20, 20),
        std::make_pair(MAG_CTRL1_REG::ODR_HZ100, 100)
    };
}

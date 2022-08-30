// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470_registers_accel.hpp"

namespace MC6470
{
std::map<uint8_t, float> acc_get_rate_hz{
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ32, 32),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ16, 16),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ8, 8),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ4, 4),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ2, 2),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ1, 1),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ0P5, 0.5),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ0P25, 0.25),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ64, 64),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ128, 128),
        std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ256, 256),
    };
} // namespace MC6470

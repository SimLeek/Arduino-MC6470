// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470.hpp"
#include "mc6470_settings.hpp"
#include <Arduino.h>

MC6470::MagMC6470 mag_state_machine(&Wire);
MC6470::AccMC6470 acc_state_machine(0, &Wire);
MC6470::MC6470Settings imu_settings = MC6470::MC6470Settings();

void setup()
{
    Serial.begin(250000);

    delay(1000); // wait for IMU to finish reading from OTP

    acc_state_machine.update_until_done();
    mag_state_machine.update_until_done();

    std::array<std::array<float, 3>, 3> mat = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    std::array<float, 3> off = { 0 };

    imu_settings.add_acc_ardu_correction(&mat, &off);
    imu_settings.add_mag_ardu_correction(&mat, &off);
    imu_settings.add_acc_res_range(MC6470::ACC_OUTCFG_REG::RANGE_4G | MC6470::ACC_OUTCFG_REG::RES_14BIT);
    imu_settings.add_acc_rate(MC6470::ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ256);

    imu_settings.write_to_eeprom();
}

void loop() { }

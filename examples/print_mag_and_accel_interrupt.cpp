// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470.hpp"
#include <Arduino.h>

MC6470::MagMC6470 mag_state_machine(&Wire);
MC6470::AccMC6470 acc_state_machine(0, &Wire);

void mag_read_callback(void*, int16_t x, int16_t y, int16_t z)
{
    Serial.print("Got mag data: x[");
    Serial.print(x);
    Serial.print("] y[");
    Serial.print(y);
    Serial.print("] z[");
    Serial.print(z);
    Serial.println("]");
}

void acc_read_callback(void*, int16_t x, int16_t y, int16_t z)
{
    Serial.print("Got acc data: x[");
    Serial.print(x);
    Serial.print("] y[");
    Serial.print(y);
    Serial.print("] z[");
    Serial.print(z);
    Serial.println("]");
}

void setup()
{
    Serial.begin(250000);

    delay(1000); // wait for IMU to finish reading from OTP

    mag_state_machine.start_async_magnetometer_read<2>(MC6470::MAG_CTRL1_REG::ODR_HZ100, mag_read_callback);
    acc_state_machine.start_accell_read_loop<3>(acc_read_callback);

    acc_state_machine.update_until_done();
    mag_state_machine.update_until_done();
}

void loop() { }

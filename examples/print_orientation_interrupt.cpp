// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470.hpp"
#include <Arduino.h>

MC6470::MagMC6470 mag_state_machine(&Wire);
MC6470::AccMC6470 acc_state_machine(0, &Wire);
MC6470::FuseMC6470 fuse_state_machine(&mag_state_machine, &acc_state_machine);

void fuse_callback(std::auto_ptr<std::array<int32_t, 9>> orient)
{
    Serial.println("Orientation:");
    Serial.print("\t");
    Serial.print((*orient)[0]);
    Serial.print(", ");
    Serial.print((*orient)[1]);
    Serial.print(", ");
    Serial.println((*orient)[2]);
    Serial.print("\t");
    Serial.print((*orient)[3]);
    Serial.print(", ");
    Serial.print((*orient)[4]);
    Serial.print(", ");
    Serial.println((*orient)[5]);
    Serial.print("\t");
    Serial.print((*orient)[6]);
    Serial.print(", ");
    Serial.print((*orient)[7]);
    Serial.print(", ");
    Serial.println((*orient)[8]);
}

void setup()
{
    Serial.begin(250000);

    delay(1000); // wait for IMU to finish reading from OTP.

    mag_state_machine.start_async_magnetometer_read<2>(
        MC6470::MAG_CTRL1_REG::ODR_HZ100, MC6470::fuse_mag_callback, &fuse_state_machine);
    acc_state_machine.start_accell_read_loop<3>(MC6470::fuse_acc_callback, &fuse_state_machine);
    fuse_state_machine.set_fusion_callback(fuse_callback);

    acc_state_machine.update_until_done();
    mag_state_machine.update_until_done();
}

void loop()
{
}

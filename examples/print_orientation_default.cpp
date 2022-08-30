// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470.hpp"
#include <Arduino.h>

MC6470::FuseMC6470 fuse_state_machine = MC6470::FuseMC6470();

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

    fuse_state_machine.set_fusion_callback(fuse_callback);
    fuse_state_machine.default_setup();
}

void loop() { fuse_state_machine.update(); }

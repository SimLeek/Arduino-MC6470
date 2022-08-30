// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470.hpp"
#include <Arduino.h>

MC6470::MagMC6470 mag_state_machine(&Wire);
MC6470::AccMC6470 acc_state_machine(0, &Wire); // 0 is default if a5 is nc
MC6470::FuseMC6470 fuse_state_machine(&mag_state_machine, &acc_state_machine);

bool report_acc = true;
bool report_mag = true;

void mag_read_callback(void*, int16_t x, int16_t y, int16_t z)
{
    fuse_state_machine.grab_mag_callback(x, y, z);
    if (report_mag) {
            Serial.print("Got mag data: x[");
            Serial.print(x);
            Serial.print("] y[");
            Serial.print(y);
            Serial.print("] z[");
            Serial.print(z);
            Serial.println("]");
    }
}

void acc_read_callback(void*, int16_t x, int16_t y, int16_t z)
{
    fuse_state_machine.grab_accel_callback(x, y, z);
    if (report_acc) {
            Serial.print("Got acc data: x[");
            Serial.print(x);
            Serial.print("] y[");
            Serial.print(y);
            Serial.print("] z[");
            Serial.print(z);
            Serial.println("]");
    }
}

void temp_read_callback(int8_t t)
{
    fuse_state_machine.grab_temp_callback(t);
    Serial.print("Got temp data: x[");
    Serial.print(t);
    Serial.println(" celcius]");
}

void acc_set_rate_callback() { Serial.println("Acc rate was set successfully."); }

void acc_set_gain_callback() { Serial.println("Acc gain and offsets were set successfully."); }

void acc_set_res_callback() { Serial.println("Acc res and range were set successfully."); }

void setup() { Serial.begin(250000); }

void loop()
{
    /*todo: make a switch case that
     *   displays an eeprom custom soft iron offset for the mag
     *   sets an eeprom custom soft iron offset for the mag
     */
    // todo: once that's done, make a python file to test it all
    char c = Serial.read();
    switch (c) {
    case 'a': {
        // start the mag read loop
        // after some quick calculations, I think the forced read can get up to 500-600hz, even with the 10ms loop wait
        mag_state_machine.start_forced_magnetometer_read(10, mag_read_callback);
        break;
    }

    case 'b': {
        // end the mag and temp read loop
        mag_state_machine.stop();
        break;
    }

    case 'c': {
        // start the acc read loop
        acc_state_machine.start_accell_read_loop(acc_read_callback);
        break;
    }

    case 'd': {
        // end the acc read loop
        acc_state_machine.stop();
        break;
    }

    // e-r deleted

    case 's': {
        while (Serial.available() <= 0) { }
        uint8_t r = (uint8_t)Serial.read();
        acc_state_machine.set_rate(r, acc_set_rate_callback);
        break;
    }

    case 't': {
        // set up a temp read loop
        mag_state_machine.start_temp_read(temp_read_callback);
        break;
    }

    case 'u': {
        uint8_t rate = acc_state_machine.get_rate();
        Serial.print("Got acc rate: ");
        Serial.println(rate);
        break;
    }

    case 'v': {
        char bytes[24];
        while (Serial.available() < 18) { }
        Serial.readBytes(bytes, 18);
        MC6470::AccMC6470::GainAndOffset* go = reinterpret_cast<MC6470::AccMC6470::GainAndOffset*>(&bytes);
        acc_state_machine.set_offset_and_gain(
            go->x_offset, go->y_offset, go->z_offset, go->x_gain, go->y_gain, go->z_gain, acc_set_gain_callback);
        break;
    }

    case 'w': {
        MC6470::AccMC6470::GainAndOffset go2 = acc_state_machine.get_gain_and_offset();
        Serial.println("Got acc offset and gain:");
        Serial.print("\t");
        Serial.print(go2.x_offset);
        Serial.print("\t");
        Serial.print(go2.y_offset);
        Serial.print("\t");
        Serial.println(go2.z_offset);
        Serial.print("\t");
        Serial.print(go2.x_gain_chip);
        Serial.print("\t");
        Serial.print(go2.y_gain_chip);
        Serial.print("\t");
        Serial.println(go2.z_gain_chip);
        break;
    }

    case 'x': {
        while (Serial.available() <= 0) { }
        uint8_t res = (uint8_t)Serial.read();
        while (Serial.available() <= 0) { }
        uint8_t range = (uint8_t)Serial.read();
        acc_state_machine.set_res_and_range(res, range, acc_set_res_callback);
        break;
    }

    case 'y': {
        uint8_t randr = acc_state_machine.get_res_and_range();
        Serial.print("Got acc res and range: ");
        Serial.println(randr);
        break;
    }

    case 'z': {
        char mbytes[6];
        while (Serial.available() < 6) { }
        Serial.readBytes(mbytes, 6);
        mag_state_machine.set_mag_offset(*reinterpret_cast<int16_t*>(mbytes), *reinterpret_cast<int16_t*>(mbytes + 2),
            *reinterpret_cast<int16_t*>(mbytes + 4));
        break;
    }

    case 'A': {
        auto moffs = mag_state_machine.get_mag_offsets();
        Serial.print("Got mag offsets: ");
        Serial.print((*moffs)[0]);
        Serial.print(", ");
        Serial.print((*moffs)[1]);
        Serial.print(", ");
        Serial.println((*moffs)[2]);
        break;
    }

    case 'B': {
        Serial.print("Ack");
        break;
    }

    case 'C': {
        report_acc = false;
        break;
    }

    case 'D': {
        report_mag = false;
        break;
    }

    case 'E': {
        report_acc = true;
        break;
    }

    case 'F': {
        report_mag = true;
        break;
    }

    case 'G': {
        char bytes[12];
        while (Serial.available() < 12) { }
        Serial.readBytes(bytes, 12);
        std::array<float, 3>* offset = reinterpret_cast<std::array<float, 3>*>(&bytes);
        mag_state_machine.set_offset(*offset);
        break;
    }

    case 'H': {
        auto moffs = mag_state_machine.get_offset();
        Serial.print("Got ardu mag offsets: ");
        Serial.print((moffs)[0]);
        Serial.print(", ");
        Serial.print((moffs)[1]);
        Serial.print(", ");
        Serial.println((moffs)[2]);
        break;
    }

    case 'I': {
        mag_state_machine.remove_offset();
        break;
    }

    case 'J': {
        char bytes[36];
        while (Serial.available() < 36) { }
        Serial.readBytes(bytes, 36);
        std::array<std::array<float, 3>, 3> mat;
        mat = *reinterpret_cast<std::array<std::array<float, 3>, 3>*>(&bytes);
        mag_state_machine.set_xform(mat);
        break;
    }

    case 'K': {
        auto mxform = mag_state_machine.get_xform();
        Serial.println("Got ardu mag xform: ");
        Serial.print("\t");
        Serial.print((mxform)[0][0]);
        Serial.print(", ");
        Serial.print((mxform)[0][1]);
        Serial.print(", ");
        Serial.println((mxform)[0][2]);

        Serial.print("\t");
        Serial.print((mxform)[1][0]);
        Serial.print(", ");
        Serial.print((mxform)[1][1]);
        Serial.print(", ");
        Serial.println((mxform)[1][2]);

        Serial.print("\t");
        Serial.print((mxform)[2][0]);
        Serial.print(", ");
        Serial.print((mxform)[2][1]);
        Serial.print(", ");
        Serial.println((mxform)[2][2]);
        break;
    }

    case 'L': {
        mag_state_machine.remove_xform();
        break;
    }

    case 'M': {
        char bytes[12];
        while (Serial.available() < 12) { }
        Serial.readBytes(bytes, 12);
        std::array<float, 3>* offset = reinterpret_cast<std::array<float, 3>*>(&bytes);
        acc_state_machine.set_offset(*offset);
        break;
    }

    case 'N': {
        auto moffs = acc_state_machine.get_offset();
        Serial.print("Got ardu acc offsets: ");
        Serial.print((moffs)[0]);
        Serial.print(", ");
        Serial.print((moffs)[1]);
        Serial.print(", ");
        Serial.println((moffs)[2]);
        break;
    }

    case 'O': {
        acc_state_machine.remove_offset();
        break;
    }

    case 'P': {
        char bytes[36];
        while (Serial.available() < 36) { }
        Serial.readBytes(bytes, 36);
        std::array<std::array<float, 3>, 3> mat;
        mat[0] = *reinterpret_cast<std::array<float, 3>*>(&bytes);
        mat[1] = *reinterpret_cast<std::array<float, 3>*>(&bytes + 12);
        mat[2] = *reinterpret_cast<std::array<float, 3>*>(&bytes + 24);
        acc_state_machine.set_xform(mat);
        break;
    }

    case 'Q': {
        auto mxform = acc_state_machine.get_xform();
        Serial.println("Got ardu acc xform: ");
        Serial.print("\t");
        Serial.print((mxform)[0][0]);
        Serial.print(", ");
        Serial.print((mxform)[0][1]);
        Serial.print(", ");
        Serial.println((mxform)[0][2]);

        Serial.print("\t");
        Serial.print((mxform)[1][0]);
        Serial.print(", ");
        Serial.print((mxform)[1][1]);
        Serial.print(", ");
        Serial.println((mxform)[1][2]);

        Serial.print("\t");
        Serial.print((mxform)[2][0]);
        Serial.print(", ");
        Serial.print((mxform)[2][1]);
        Serial.print(", ");
        Serial.println((mxform)[2][2]);
        break;
    }

    case 'R': {
        acc_state_machine.remove_xform();
        break;
    }

    default:
        break;
    }
    mag_state_machine.update();
    acc_state_machine.update();
}

#include <Arduino.h>
#include "mc6470.hpp"

MC6470::MagMC6470 mag_state_machine(Wire);
MC6470::AccMC6470 acc_state_machine(1, Wire);
MC6470::FuseMC6470 fuse_state_machine(mag_state_machine, acc_state_machine);

bool report_acc_in_gs = false;
bool report_mag_in_el = false;

void mag_read_callback(int16_t x, int16_t y, int16_t z)
{
  fuse_state_machine.required_grab_mag_callback(x, y, z);
  if (report_mag_in_el)
  {
    x = x * mag_state_machine.mag_to_earth_local;
    y = y * mag_state_machine.mag_to_earth_local;
    z = z * mag_state_machine.mag_to_earth_local;
  }
  Serial.print("Got mag data: x[");
  Serial.print(x);
  Serial.print("] y[");
  Serial.print(y);
  Serial.print("] z[");
  Serial.print(z);
  Serial.println("]");
}

void acc_read_callback(int16_t x, int16_t y, int16_t z)
{
  fuse_state_machine.required_grab_accel_callback(x, y, z);
  if (report_acc_in_gs)
  {
    x = x * acc_state_machine.acc_to_g;
    y = y * acc_state_machine.acc_to_g;
    z = z * acc_state_machine.acc_to_g;
  }
  Serial.print("Got acc data: x[");
  Serial.print(x);
  Serial.print("] y[");
  Serial.print(y);
  Serial.print("] z[");
  Serial.print(z);
  Serial.println("]");
}

void temp_read_callback(int8_t t)
{
  fuse_state_machine.required_grab_temp_callback(t);
  Serial.print("Got temp data: x[");
  Serial.print(t);
  Serial.println(" celcius]");
}

void acc_set_rate_callback()
{
  Serial.println("Acc rate was set successfully.");
}

void acc_set_gain_callback()
{
  Serial.println("Acc gain and offsets were set successfully.");
}

void acc_set_res_callback()
{
  Serial.println("Acc res and range were set successfully.");
}

void setup()
{
  Serial.begin(250000);
}

void loop()
{
  /*todo: make a switch case that
   *   displays the offset/gain values for acc
   *   sets the offset/gain values for acc
   *   displays the offset values for the mag
   *   sets the offset values for the mag
   *   set acc loop speed
   *   read acc loop speed
   *   set mag loop speed
   *   read mag loop speed
   *   set acc resolution
   *   read acc resolution
   *   set mag resolution
   *   read mag resolution
   *   displays an eeprom custom soft iron offset for the mag
   *   sets an eeprom custom soft iron offset for the mag
   */
  // todo: once that's done, make a python file to test it all
  char c = Serial.read();
  switch (c)
  {
  case 'a':
  {
    // start the mag read loop
    mag_state_machine.start_forced_magnetometer_read(10, mag_read_callback);
    break;
  }

  case 'b':
  {
    // end the mag and temp read loop
    mag_state_machine.stop();
    break;
  }

  case 'c':
  {
    // start the acc read loop
    acc_state_machine.start_accell_read_loop(acc_read_callback);
    break;
  }

  case 'd':
  {
    // end the acc read loop
    acc_state_machine.stop();
    break;
  }

  case 'e':
  {
    // show fusion
    fuse_state_machine.display_fusion_update = true;
    break;
  }

  case 'f':
  {
    // stop showing fusion
    fuse_state_machine.display_fusion_update = false;
    break;
  }

  case 'g':
  {
    acc_state_machine.reset_g_gather();
    acc_state_machine.start_g_gather();
    break;
  }

  case 'h':
  {
    acc_state_machine.stop_g_gather();
    break;
  }

  case 'i':
  {
    mag_state_machine.reset_el_gather();
    mag_state_machine.start_el_gather();
    break;
  }

  case 'j':
  {
    mag_state_machine.stop_el_gather();
    break;
  }

  case 'k':
  {
    Serial.print("acc_to_g: ");
    Serial.println(acc_state_machine.acc_to_g);
    break;
  }

  case 'l':
  {
    Serial.print("acc_to_g_var: ");
    Serial.println(acc_state_machine.acc_to_g_variance);
    break;
  }

  case 'm':
  {
    Serial.print("mag_to_earth_local: ");
    Serial.println(mag_state_machine.mag_to_earth_local);
    break;
  }

  case 'n':
  {
    Serial.print("mag_to_earth_local_variance: ");
    Serial.println(mag_state_machine.mag_to_earth_local_variance);
    break;
  }

  case 'o':
  {
    report_acc_in_gs = true;
    break;
  }

  case 'p':
  {
    report_acc_in_gs = false;
    break;
  }

  case 'q':
  {
    report_mag_in_el = true;
    break;
  }

  case 'r':
  {
    report_mag_in_el = false;
    break;
  }

  case 's':
  {
    uint8_t r = (uint8_t)Serial.read();
    acc_state_machine.set_rate(r, acc_set_rate_callback);
    break;
  }

  case 't':
  {
    // set up a temp read loop
    mag_state_machine.start_temp_read(temp_read_callback);
    break;
  }

  case 'u':
  {
    uint8_t rate = acc_state_machine.get_rate();
    Serial.print("Got acc rate: ");
    Serial.println(rate);
    break;
  }

  case 'v':
  {
    char bytes[24];
    Serial.readBytes(bytes, 18);
    MC6470::AccMC6470::GainAndOffset *go = reinterpret_cast<MC6470::AccMC6470::GainAndOffset *>(&bytes);
    acc_state_machine.set_offset_and_gain(
        go->x_offset, go->y_offset, go->z_offset,
        go->x_gain, go->y_gain, go->z_gain,
        acc_set_gain_callback);
    break;
  }

  case 'w':
  {
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

  case 'x':
  {
    uint8_t res = (uint8_t)Serial.read();
    uint8_t range = (uint8_t)Serial.read();
    acc_state_machine.set_res_and_range(res, range, acc_set_res_callback);
    break;
  }

  case 'y':
  {
    uint8_t randr = acc_state_machine.get_res_and_range();
    Serial.print("Got acc res and range: ");
    Serial.println(randr);
    break;
  }

  case 'z':
  {
    char mbytes[6];
    Serial.readBytes(mbytes, 6);
    mag_state_machine.set_mag_offset(
        *(int16_t *)mbytes,
        *(int16_t *)(mbytes + 2),
        *(int16_t *)(mbytes + 4));
    break;
  }

  case 'A':
  {
    auto moffs = mag_state_machine.get_mag_offsets();
    Serial.print("Got mag offsets: ");
    Serial.print((*moffs)[0]);
    Serial.print(", ");
    Serial.print((*moffs)[1]);
    Serial.print(", ");
    Serial.println((*moffs)[2]);
    break;
  }

  default:
    break;
  }
  mag_state_machine.run_state();
}

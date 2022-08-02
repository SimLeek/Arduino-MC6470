#include <Arduino.h>
#include "mc6470.hpp"

MC6470::MagMC6470 state_machine(Wire);

void mag_read_callback(int16_t x, int16_t y, int16_t z){
  Serial.print("Got mag data: x[");
  Serial.print(x);
  Serial.print("] y[");
  Serial.print(y);
  Serial.print("] z[");
  Serial.print(z);
  Serial.println("]");
}

void setup() {
  Serial.begin(250000);
  state_machine.start_forced_magnetometer_read(10, mag_read_callback);
}

void loop() {
  state_machine.run_state();
}


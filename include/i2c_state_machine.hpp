#ifndef I2C_STATE_MACHINE_HPP
#define I2C_STATE_MACHINE_HPP

#include <Arduino.h>
#include <Wire.h>
#include "state_machine.hpp"

class I2CStateMachine : public StateMachine{

public:

  I2CStateMachine(TwoWire& _wire):wire(_wire){
    func_print;
  }

  /********************
   GENERAL I2C STUFF
  *********************/

  uint8_t read_byte(uint8_t dev_id, uint8_t reg_addr){
    func_print;
    wire.beginTransmission(dev_id);
    wire.write(reg_addr);
    wire.endTransmission();
    wire.requestFrom((int)dev_id, 1);
    return (uint8_t)wire.read();
  }

  uint16_t read_2byte(uint8_t dev_id, uint8_t reg_addr){
    func_print;
    //arduino matches endian of mag data, so just placing like this should work
    uint8_t lsb = read_byte(dev_id, reg_addr);
    uint8_t msb = read_byte(dev_id, reg_addr);
    int16_t num = 0 | lsb | msb<<8;
    return num;
  }

  uint8_t* burst_read_bytes(uint8_t dev_id, uint8_t reg_addr, uint8_t read_len){
    //calling function must delete the uint8 mem.
    func_print;
    //arduino matches endian of mag data, so just placing like this should work
    uint8_t* bytes = new uint8_t[read_len];
    
    wire.beginTransmission(dev_id);
    wire.write(reg_addr);
    wire.endTransmission();
    wire.requestFrom(dev_id, read_len);

    for(uint8_t i=0; i<read_len; i++){
      bytes[i] = (uint8_t)wire.read();
    }

    return bytes;
  }

  void write_byte(uint8_t dev_id, uint8_t reg_addr, uint8_t val){
    func_print;
    wire.beginTransmission(dev_id);
    wire.write(write_reg);
    wire.write(val);
    wire.endTransmission();
  }

 void wait(unsigned long ms, state_machine_state next)
    {
      func_print;
        wait_time = ms;
        state_after_wait = next;
        check_time = millis();
        set_state(wait_state);
    }

    void wait_state()
    {
      func_print;
        if (millis() - check_time > wait_time)
        {
            this->current_state_function = this->state_after_wait;
        }
    }

  void write_until_read_back(
    uint8_t address, 
    uint8_t reg, 
    uint8_t val, 
    uint8_t mask, 
    state_machine_state next_state,
    unsigned int timeout=1000,
    state_machine_state timeout_state=nullptr
    ){
      func_print;
    write_address = address;
    write_reg = reg;
    write_val = val;
    write_mask = mask;
    state_after_readback = next_state;
    wait_time = timeout;
    state_after_wait = timeout_state;
    check_time = millis();

    set_state(write_until_read_back_state);
  }


  void write_until_read_back_state(){
    func_print;
    uint8_t response = read_byte(write_address, write_reg);

    uint8_t to_write = (response&~write_mask)|(write_val&write_mask);

    if (to_write==response){
      this->current_state_function = state_after_readback;
      return;
    }

    write_byte(write_address, write_reg, to_write);

    if ((millis() - check_time > wait_time) && this->state_after_wait!=nullptr)
    {
        this->current_state_function = this->state_after_wait;
    }
  }

  uint8_t write_address;
  uint8_t write_reg;
  uint8_t write_val;
  uint8_t write_mask;
  state_machine_state state_after_readback;

  void read_until_register_change(){}

  void ping_until_register_change(uint8_t address, uint8_t reg, uint8_t val, 
    uint8_t mask, 
    state_machine_state next_state,
    unsigned int ping_wait_ms=2){
      func_print;

    write_address = address;
    write_reg = reg;
    write_val = val;
    write_mask = mask;
    state_after_readback = next_state;
    wait_time = ping_wait_ms;
    //state_after_wait = timeout_state;
    check_time = millis()-wait_time-1;

    set_state(write_until_read_back_state);

    }

    void ping_until_register_change_state(){
      func_print;
      if (millis() - check_time > wait_time)
        {
          uint8_t response = read_byte(write_address, write_reg);

          uint8_t expected = (response&~write_mask)|(write_val&write_mask);

          if (expected==response){
            this->current_state_function = state_after_readback;
            return;
          }

          check_time = millis();
        }      
    }

    TwoWire& wire;

};

#endif
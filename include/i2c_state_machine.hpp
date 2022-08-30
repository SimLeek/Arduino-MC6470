// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#ifndef INCLUDE_I2C_STATE_MACHINE_HPP_
#define INCLUDE_I2C_STATE_MACHINE_HPP_

#include <wire.h>
#include "state_machine.hpp"
#include <Arduino.h>


#define cast_state_i2c(x) reinterpret_cast<state_machine_state>(&I2CStateMachine::x)
#define set_state_i2c(x) this->current_state_function = cast_state_i2c(x)

class I2CStateMachine : public StateMachine {
public:
    explicit I2CStateMachine(TwoWire* _wire)
        : wire(_wire)
    {
    }

    /********************
     GENERAL I2C STUFF
    *********************/

    uint8_t read_byte(uint8_t dev_id, uint8_t reg_addr);

    uint16_t read_2byte(uint8_t dev_id, uint8_t reg_addr);

    void write_2byte(uint8_t dev_id, uint8_t reg_addr, int16_t val);

    uint8_t* burst_read_bytes(uint8_t dev_id, uint8_t reg_addr, uint8_t read_len);

    void write_byte(uint8_t dev_id, uint8_t reg_addr, uint8_t val);

    void write_until_read_back(uint8_t address, uint8_t reg, uint8_t val, uint8_t mask, state_machine_state next_state,
        unsigned int timeout = 1000, state_machine_state timeout_state = nullptr);

    void write_byte_with_mask(uint8_t dev_id, uint8_t reg_addr, uint8_t val, uint8_t mask);

    void write_until_read_back_state();

    uint8_t write_address = 0;
    uint8_t write_reg = 0;
    uint8_t write_val = 0;
    uint8_t write_mask = 0;
    state_machine_state state_after_readback = cast_state_i2c(noop_state);

    void ping_until_register_change(uint8_t address, uint8_t reg, uint8_t val, uint8_t mask,
        state_machine_state next_state, unsigned int ping_wait_ms = 2);

    void ping_until_register_change_state();

    TwoWire* wire;
};

#endif // INCLUDE_I2C_STATE_MACHINE_HPP_

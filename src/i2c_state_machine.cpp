// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "i2c_state_machine.hpp"

uint8_t I2CStateMachine::read_byte(uint8_t dev_id, uint8_t reg_addr)
{
    wire->beginTransmission(dev_id);
    wire->write(reg_addr);
    wire->endTransmission();
    wire->requestFrom(static_cast<int>(dev_id), 1);
    return (uint8_t)wire->read();
}

uint16_t I2CStateMachine::read_2byte(uint8_t dev_id, uint8_t reg_addr)
{
    // arduino matches endian of mag data, so just placing like this should work
    uint8_t lsb = read_byte(dev_id, reg_addr);
    uint8_t msb = read_byte(dev_id, reg_addr + 1);
    int16_t num = 0 | lsb | msb << 8;
    return num;
}

void I2CStateMachine::write_2byte(uint8_t dev_id, uint8_t reg_addr, int16_t val)
{
    // arduino matches endian of mag data, so just placing like this should work
    uint8_t lsb = val & (uint8_t)(-1);
    uint8_t msb = (uint8_t)(val >> 8);
    write_byte(dev_id, reg_addr, lsb);
    write_byte(dev_id, reg_addr + 1, msb);
    return;
}

uint8_t* I2CStateMachine::burst_read_bytes(uint8_t dev_id, uint8_t reg_addr, uint8_t read_len)
{
    // calling function must delete the uint8 mem.

    // arduino matches endian of mag data, so just placing like this should work
    uint8_t* bytes = new uint8_t[read_len];

    wire->beginTransmission(dev_id);
    wire->write(reg_addr);
    wire->endTransmission();
    wire->requestFrom(dev_id, read_len);

    for (uint8_t i = 0; i < read_len; i++) {
        bytes[i] = (uint8_t)wire->read();
    }

    return bytes;
}

void I2CStateMachine::write_byte(uint8_t dev_id, uint8_t reg_addr, uint8_t val)
{
    wire->beginTransmission(dev_id);
    wire->write(write_reg);
    wire->write(val);
    wire->endTransmission();
}

void I2CStateMachine::write_until_read_back(uint8_t address, uint8_t reg, uint8_t val, uint8_t mask,
    state_machine_state next_state, unsigned int timeout, state_machine_state timeout_state)
{
    write_address = address;
    write_reg = reg;
    write_val = val;
    write_mask = mask;
    state_after_readback = next_state;
    wait_time = timeout;
    state_after_wait = timeout_state;
    check_time = millis();

    set_state_i2c(write_until_read_back_state);
}

void I2CStateMachine::write_byte_with_mask(uint8_t dev_id, uint8_t reg_addr, uint8_t val, uint8_t mask)
{
    uint8_t response = read_byte(dev_id, reg_addr);
    uint8_t to_write = (response & ~mask) | (val & mask);
    wire->beginTransmission(dev_id);
    wire->write(reg_addr);
    wire->write(to_write);
    wire->endTransmission();
}

void I2CStateMachine::write_until_read_back_state()
{
    uint8_t response = read_byte(write_address, write_reg);
    // Serial.print("wurb response: "); Serial.println(response);
    // Serial.print("wurb write_val: "); Serial.println(write_val);
    uint8_t to_write = (response & ~write_mask) | (write_val & write_mask);
    // Serial.print("wurb to_write: "); Serial.println(response);
    if (to_write == response) {
        this->current_state_function = state_after_readback;
        return;
    }

    write_byte(write_address, write_reg, to_write);

    if ((millis() - check_time > wait_time) && this->state_after_wait != nullptr) {
        this->current_state_function = this->state_after_wait;
    }
}

void I2CStateMachine::ping_until_register_change(
    uint8_t address, uint8_t reg, uint8_t val, uint8_t mask, state_machine_state next_state, unsigned int ping_wait_ms)
{
    write_address = address;
    write_reg = reg;
    write_val = val;
    write_mask = mask;
    state_after_readback = next_state;
    wait_time = ping_wait_ms;
    // state_after_wait = timeout_state;
    check_time = millis() - wait_time - 1;

    set_state_i2c(ping_until_register_change_state);
}

void I2CStateMachine::ping_until_register_change_state()
{
    if (millis() - check_time > wait_time) {
        uint8_t response = read_byte(write_address, write_reg);

        uint8_t expected = (response & ~write_mask) | (write_val & write_mask);

        if (expected == response) {
            this->current_state_function = state_after_readback;
            return;
        }

        check_time = millis();
    }
}

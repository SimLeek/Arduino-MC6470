// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470_accel.hpp"

namespace MC6470 {

AccMC6470::AccMC6470(int a5_state, TwoWire* _wire)
    : I2CStateMachine(_wire)
{
    set_address_from_a5(a5_state);
    set_state_acc(noop_state);
}

void AccMC6470::set_address_from_a5(int a5_state)
{
    // Adding this as a function in case someone wants to switch addresses while the device is running
    if (a5_state) {
        this->address_accel = address_accel_a5vcc;
    } else {
        this->address_accel = address_accel_a5gnd;
    }
}

void AccMC6470::stop() { set_state_acc(noop_state); }

#pragma region GENERIC STATES

void AccMC6470::ensure_standby_mode()
{
    write_until_read_back(address_accel, acc_mode_reg_address, ACC_MODE_REG::OPCON_STANDBY, ACC_MODE_REG::OPCON_MASK,
        generic_state_queue.front());
    generic_state_queue.pop();
}

void AccMC6470::check_standby_and_otp()
{
    uint8_t full_val = ACC_OPSTAT_REG::OPSTAT_STANDBY | ACC_OPSTAT_REG::OTPA_IDLE;
    uint8_t full_mask = ACC_OPSTAT_REG::OPSTAT_MASK | ACC_OPSTAT_REG::OTPA_MASK;
    ping_until_register_change(
        address_accel, acc_opstat_reg_address, full_val, full_mask, generic_state_queue.front(), 2);
    generic_state_queue.pop();
}

void AccMC6470::call_success_callback()
{
    this->on_success_callback();
    set_state_acc(noop_state);
}

#pragma endregion
#pragma region SET RESOLUTION AND RANGE

void AccMC6470::set_res_and_range(uint8_t target_res, uint8_t _target_range, void (*callback_on_success)())
{
    on_success_callback = callback_on_success;
    target_resolution = target_res;
    target_range = _target_range;
    set_state_acc(ensure_standby_mode);
    generic_state_queue.push(cast_state_acc(check_standby_and_otp));
    generic_state_queue.push(cast_state_acc(set_res_and_range_state));
}

uint8_t AccMC6470::get_res_and_range()
{
    uint8_t res_range_byte = read_byte(address_accel, acc_outcfg_reg_address);
    return res_range_byte;
}

void AccMC6470::set_res_and_range_state()
{
    uint8_t full_val = target_range | target_resolution;
    uint8_t full_mask = ACC_OUTCFG_REG::RANGE_MASK | ACC_OUTCFG_REG::RES_MASK;
    write_until_read_back(address_accel, acc_outcfg_reg_address, full_val, full_mask,
        cast_state_acc(call_success_callback), 1000, cast_state_acc(noop_state));
}

#pragma endregion
#pragma region SET GAINS AND OFFSETS

void AccMC6470::set_offset_and_gain(int16_t off_x, int16_t off_y, int16_t off_z, float gain_x, float gain_y,
    float gain_z, void (*callback_on_success)())
{
    on_success_callback = callback_on_success;
    target.x_offset = off_x;
    target.y_offset = off_y;
    target.z_offset = off_z;
    target.x_gain = gain_x;
    target.y_gain = gain_y;
    target.z_gain = gain_z;
    set_state_acc(ensure_standby_mode);
    generic_state_queue.push(cast_state_acc(check_standby_and_otp));
    generic_state_queue.push(cast_state_acc(set_gain_and_offset_state));
}

AccMC6470::GainAndOffset AccMC6470::get_gain_and_offset()
{
    GainAndOffset current;

    uint8_t xoff_lsb = read_byte(address_accel, 0x21);
    uint8_t xoff_msb = read_byte(address_accel, 0x22);
    uint8_t yoff_lsb = read_byte(address_accel, 0x23);
    uint8_t yoff_msb = read_byte(address_accel, 0x24);
    uint8_t zoff_lsb = read_byte(address_accel, 0x25);
    uint8_t zoff_msb = read_byte(address_accel, 0x26);
    uint8_t xgain = read_byte(address_accel, 0x27);
    uint8_t ygain = read_byte(address_accel, 0x28);
    uint8_t zgain = read_byte(address_accel, 0x29);

    current.x_offset = ((uint16_t)0 | xoff_lsb | ((uint16_t)(xoff_msb & 0b01111111)) << 8) << 1 / 2;
    current.y_offset = ((uint16_t)0 | yoff_lsb | ((uint16_t)(yoff_msb & 0b01111111)) << 8) << 1 / 2;
    current.z_offset = ((uint16_t)0 | zoff_lsb | ((uint16_t)(zoff_msb & 0b01111111)) << 8) << 1 / 2;

    current.x_gain_chip = (uint16_t)0 | xgain | ((uint16_t)(xoff_msb & 0b10000000)) << 1;
    current.y_gain_chip = (uint16_t)0 | ygain | ((uint16_t)(yoff_msb & 0b10000000)) << 1;
    current.z_gain_chip = (uint16_t)0 | zgain | ((uint16_t)(zoff_msb & 0b10000000)) << 1;

    return current;
}

void AccMC6470::set_gain_and_offset_state()
{
    GainAndOffset to_send = get_gain_and_offset();
    to_send.x_offset = add_with_limits_signed<int16_t, 15>(to_send.x_offset, target.x_offset);
    to_send.y_offset = add_with_limits_signed<int16_t, 15>(to_send.y_offset, target.y_offset);
    to_send.z_offset = add_with_limits_signed<int16_t, 15>(to_send.z_offset, target.z_offset);
    to_send.x_gain_chip = multiply_with_limits_unsigned<uint16_t, 9, float>(to_send.x_gain_chip, target.x_gain);
    to_send.y_gain_chip = multiply_with_limits_unsigned<uint16_t, 9, float>(to_send.y_gain_chip, target.y_gain);
    to_send.z_gain_chip = multiply_with_limits_unsigned<uint16_t, 9, float>(to_send.z_gain_chip, target.z_gain);

    write_byte(address_accel, 0x20, (uint8_t)to_send.x_offset);
    write_byte(
        address_accel, 0x21, ((uint8_t)(((uint16_t)to_send.x_offset) >> 8) & 0b01111111) | to_send.x_gain_chip >> 8);
    write_byte(address_accel, 0x22, (uint8_t)to_send.y_offset);
    write_byte(
        address_accel, 0x23, ((uint8_t)(((uint16_t)to_send.y_offset) >> 8) & 0b01111111) | to_send.y_gain_chip >> 8);
    write_byte(address_accel, 0x24, (uint8_t)to_send.z_offset);
    write_byte(
        address_accel, 0x25, ((uint8_t)(((uint16_t)to_send.z_offset) >> 8) & 0b01111111) | to_send.z_gain_chip >> 8);
    write_byte(address_accel, 0x26, (uint8_t)to_send.x_gain_chip);
    write_byte(address_accel, 0x27, (uint8_t)to_send.y_gain_chip);
    write_byte(address_accel, 0x28, (uint8_t)to_send.z_gain_chip);

    set_state_acc(noop_state);
    if (on_success_callback != nullptr)
        this->on_success_callback();
}

#pragma endregion
#pragma region SET POLLING RATE

void AccMC6470::set_rate(uint8_t _target_rate, void (*callback_on_success)())
{
    on_success_callback = callback_on_success;
    target_rate = _target_rate;
    set_state_acc(ensure_standby_mode);
    generic_state_queue.push(cast_state_acc(check_standby_and_otp));
    generic_state_queue.push(cast_state_acc(set_rate_state));
}

uint8_t AccMC6470::get_rate()
{
    uint8_t rate = read_byte(address_accel, acc_srtf_reg_address);
    return rate;
}

void AccMC6470::set_rate_state()
{
    write_until_read_back(address_accel, acc_srtf_reg_address, target_rate,
        ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_MASK, cast_state_acc(call_success_callback), 1000,
        cast_state_acc(noop_state));
}

#pragma endregion
#pragma region READ LOOP

void AccMC6470::set_offset(std::array<float, 3> new_offset)
{
    offset_xyz = std::move(new_offset);
    has_offset = true;
}

const std::array<float, 3>& AccMC6470::get_offset() const& { return offset_xyz; }

void AccMC6470::remove_offset() { has_offset = false; }

void AccMC6470::set_xform(std::array<std::array<float, 3>, 3> new_xform)
{
    lin_xform = std::move(new_xform);
    has_xform = true;
}

const std::array<std::array<float, 3>, 3>& AccMC6470::get_xform() const& { return lin_xform; }

void AccMC6470::remove_xform() { has_xform = false; }

void AccMC6470::ensure_inta_on_new_data()
{
    write_until_read_back(address_accel, acc_inten_reg_address, ACC_INTEN_REG::ACQ_INT_EN, ACC_INTEN_REG::ACQ_INT_EN,
        generic_state_queue.front());
    generic_state_queue.pop();
}

void AccMC6470::ensure_wake_mode()
{
    write_until_read_back(address_accel, acc_mode_reg_address, ACC_MODE_REG::OPCON_WAKE, ACC_MODE_REG::OPCON_MASK,
        generic_state_queue.front());
    generic_state_queue.pop();
}

void AccMC6470::check_wake_and_otp()
{
    uint8_t full_val = ACC_OPSTAT_REG::OPSTAT_WAKE | ACC_OPSTAT_REG::OTPA_IDLE;
    uint8_t full_mask = ACC_OPSTAT_REG::OPSTAT_MASK | ACC_OPSTAT_REG::OTPA_MASK;
    ping_until_register_change(
        address_accel, acc_opstat_reg_address, full_val, full_mask, generic_state_queue.front(), 2);
    generic_state_queue.pop();
}

void AccMC6470::check_acq_int()
{
    mag_read_loop_wait_ms = (unsigned int)((1000 / acc_get_rate_hz[target_rate]) / 2);
    ping_until_register_change(address_accel, acc_sr_reg_address, ACC_SR_REG::ACQ_INT, ACC_SR_REG::ACQ_INT,
        cast_state_acc(read_acc_xyz), mag_read_loop_wait_ms);
}

void AccMC6470::read_acc_xyz()
{
    int16_t x = read_2byte(address_accel, acc_xyz_reg_address);
    int16_t y = read_2byte(address_accel, acc_xyz_reg_address + 2);
    int16_t z = read_2byte(address_accel, acc_xyz_reg_address + 4);

    std::array<int16_t, 3> xyz({ x, y, z });

    if (has_offset) {
        auto fxyz = Add(xyz, offset_xyz);
        xyz[0] = (int16_t)fxyz[0];
        xyz[1] = (int16_t)fxyz[1];
        xyz[2] = (int16_t)fxyz[2];
    }

    if (has_xform) {
        auto fxyz = MatMul3331(lin_xform, xyz);
        xyz[0] = (int16_t)fxyz[0];
        xyz[1] = (int16_t)fxyz[1];
        xyz[2] = (int16_t)fxyz[2];
    }

    if (has_offset || has_xform) {
        x = xyz[0];
        y = xyz[1];
        z = xyz[2];
    }

    acc_read_callback(acc_read_context, x, y, z);
    if (interrupt_pin == -1)
        set_state_acc(check_acq_int);
    else
        set_state_acc(noop_state);

    // free(bytes);
}

#pragma endregion

} // namespace MC6470

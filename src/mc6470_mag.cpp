// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470_mag.hpp"

namespace MC6470 {

MagMC6470::MagMC6470(TwoWire* _wire)
    : I2CStateMachine(_wire)
{
    set_state_mag(noop_state);
}

void MagMC6470::call_success_callback()
{
    this->on_success_callback();
    set_state_mag(noop_state);
}

void MagMC6470::set_dynamic_range(uint8_t range)
{
    write_byte_with_mask(address_mag, mag_range_reg_address, range, MAG_RANGE_REG::RANGE_15BIT);
}

void MagMC6470::stop() { set_state_mag(noop_state); }

#pragma region GENERIC STATES

void MagMC6470::ensure_pc_generic_state()
{
    write_until_read_back(
        address_mag, mag_ctrl1_reg_address, MAG_CTRL1_REG::PC, MAG_CTRL1_REG::PC, generic_state_queue.front());
    generic_state_queue.pop();
}

void MagMC6470::ensure_standby_generic_state()
{
    write_until_read_back(address_mag, mag_ctrl1_reg_address, 0, MAG_CTRL1_REG::PC, generic_state_queue.front());
    generic_state_queue.pop();
}

void MagMC6470::ensure_fs_generic_state()
{
    write_until_read_back(
        address_mag, mag_ctrl1_reg_address, MAG_CTRL1_REG::FS, MAG_CTRL1_REG::FS, generic_state_queue.front());
    generic_state_queue.pop();
}

void MagMC6470::ensure_fs_normal_generic_state()
{
    write_until_read_back(address_mag, mag_ctrl1_reg_address, 0, MAG_CTRL1_REG::FS, generic_state_queue.front());
    generic_state_queue.pop();
}

void MagMC6470::ensure_ocl_generic_state()
{
    write_until_read_back(
        address_mag, mag_ctrl3_reg_address, MAG_CTRL3_REG::OCL, MAG_CTRL3_REG::OCL, generic_state_queue.front());
    generic_state_queue.pop();
}

void MagMC6470::ensure_ocl_normal_generic_state()
{
    write_until_read_back(address_mag, mag_ctrl3_reg_address, 0, MAG_CTRL3_REG::OCL, generic_state_queue.front());
    generic_state_queue.pop();
}

void MagMC6470::ensure_frc_generic_state()
{
    write_until_read_back(
        address_mag, mag_ctrl3_reg_address, MAG_CTRL3_REG::FORCE, MAG_CTRL3_REG::FORCE, generic_state_queue.front());
    generic_state_queue.pop();
}

#pragma endregion
#pragma region ASYNC READ LOOP

void MagMC6470::set_intm_on_state()
{
    write_until_read_back(
        address_mag, mag_ctrl2_reg_address, MAG_CTRL2_REG::DEN, MAG_CTRL2_REG::DEN, generic_state_queue.front());
}

void MagMC6470::begin_async_mag_state()
{
    write_until_read_back(address_mag, mag_ctrl1_reg_address, MAG_CTRL1_REG::PC | async_mag_rate,
        MAG_CTRL1_REG::PC | MAG_CTRL1_REG::ODR_HZ100, generic_state_queue.front());
}

#pragma endregion
#pragma region FORCED READ LOOP

void MagMC6470::set_mag_read_callback(void (*_mag_read_callback)(void*, int16_t, int16_t, int16_t))
{
    mag_read_callback = _mag_read_callback;
}

void MagMC6470::set_offset(std::array<float, 3> new_offset)
{
    offset_xyz = std::move(new_offset);
    has_offset = true;
}

const std::array<float, 3>& MagMC6470::get_offset() const& { return offset_xyz; }

void MagMC6470::remove_offset() { has_offset = false; }

void MagMC6470::set_xform(std::array<std::array<float, 3>, 3> new_xform)
{
    lin_xform = std::move(new_xform);
    has_xform = true;
}

const std::array<std::array<float, 3>, 3>& MagMC6470::get_xform() const& { return lin_xform; }

void MagMC6470::remove_xform() { has_xform = false; }

void MagMC6470::ensure_not_frc_and_read_mag_state()
{
    // This sometimes fails/times out, but retrying works and makes the reads work 100% of the time.
    if (generic_state_queue.size() == 0)
        generic_state_queue.push(cast_state_mag(ensure_not_frc_and_read_mag_state));
    write_until_read_back(address_mag, mag_ctrl3_reg_address, ~MAG_CTRL3_REG::FORCE, MAG_CTRL3_REG::FORCE,
        cast_state_mag(check_drdy_then_read_mag_state), 100, cast_state_mag(ensure_frc_generic_state));
}

void MagMC6470::check_drdy_then_read_mag_state()
{
    if (interrupt_pin == -1)
        ping_until_register_change(address_mag, mag_status_reg_address, MAG_STATUS_REG::DRDY, MAG_STATUS_REG::DRDY,
            cast_state_mag(finally_read_mag_state), 2);
    else
        set_state_mag(noop_state);
}

void MagMC6470::finally_read_mag_state()
{
    // todo: turn this into a func, read individual bytes back, and build up mag xyz
    int16_t x = read_2byte(address_mag, 0x10);
    int16_t y = read_2byte(address_mag, 0x12);
    int16_t z = read_2byte(address_mag, 0x14);

    // x = (x<<(16-mag_bits))/((uint16_t)(1)<<(16-mag_bits));
    // y = (y<<(16-mag_bits))/((uint16_t)(1)<<(16-mag_bits));
    // y = (z<<(16-mag_bits))/((uint16_t)(1)<<(16-mag_bits));

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

    mag_read_callback(mag_read_context, x, y, z);

    if (async_reading) {
        wait(mag_read_loop_wait_ms, cast_state_mag(check_drdy_then_read_mag_state));
    } else {
        if (mag_read_loop_wait_ms)
            wait(mag_read_loop_wait_ms, cast_state_mag(ensure_frc_generic_state));
        else
            set_state_mag(ensure_frc_generic_state);
        generic_state_queue.push(cast_state_mag(ensure_not_frc_and_read_mag_state));
    }
}

#pragma endregion

#pragma region TEMP READ STATE

void MagMC6470::ensure_tcs_generic_state()
{
    write_byte_with_mask(address_mag, mag_ctrl3_reg_address, MAG_CTRL3_REG::TCS, MAG_CTRL3_REG::TCS);
    ping_until_register_change(
        address_mag, mag_ctrl3_reg_address, ~MAG_CTRL3_REG::TCS, MAG_CTRL3_REG::TCS, generic_state_queue.front(), 2);
    generic_state_queue.pop();
}

void MagMC6470::finally_read_temp_state()
{
    // todo: turn this into a func, read individual bytes back, and build up mag xyz
    int8_t t = read_byte(address_mag, 0x31);

    set_state_mag(noop_state);

    temp_read_callback(t);
}

#pragma endregion

#pragma region SET MAG OFFSET

void MagMC6470::set_mag_offset(int16_t x, int16_t y, int16_t z)
{
    // todo: ping drdy time could be modified as well, on top of loop wait. Currently ping drdy time is set to 2 ms.
    // todo: check replacing ensure_not_frc with ping_until_change of 1ms and check if it hangs
    // todo: add option to replace ensure pc and ensure fs with in-memory checks for slight speedup? ...is it worth
    // it though? may be faster than the max 100hz. not tested.
    target_mag_offset_x = x;
    target_mag_offset_y = y;
    target_mag_offset_z = z;

    Serial.print("requested set mag offsets: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);

    set_state_mag(ensure_pc_generic_state);
    // todo: do fs and tcs in one so it automatically goes back after measurement
    generic_state_queue.push(cast_state_mag(ensure_fs_generic_state));
    // generic_state_queue.push(cast_state_mag(ensure_ocl_generic_state));
    generic_state_queue.push(cast_state_mag(finally_set_offset_state));
}

std::auto_ptr<std::array<int16_t, 3>> MagMC6470::get_mag_offsets()
{
    int16_t* c = new int16_t[3];

    c[0] = read_2byte(address_mag, 0x20);
    c[1] = read_2byte(address_mag, 0x22);
    c[2] = read_2byte(address_mag, 0x24);

    std::array<int16_t, 3>* A = reinterpret_cast<std::array<int16_t, 3>*>(c);
    std::auto_ptr<std::array<int16_t, 3>> a_ptr(A);
    return a_ptr;
}

void MagMC6470::finally_set_offset_state()
{
    // write_byte_with_mask(address_mag, mag_ctrl3_reg_address, MAG_CTRL3_REG::OCL, MAG_CTRL3_REG::OCL);
    //  if the mag had offsets already, what we measured included those.

    auto offsets = get_mag_offsets();
    write_2byte(address_mag, 0x20, (*offsets)[0] + target_mag_offset_x);
    write_2byte(address_mag, 0x22, (*offsets)[1] + target_mag_offset_y);
    write_2byte(address_mag, 0x24, (*offsets)[2] + target_mag_offset_z);
    // write_until_read_back(address_mag, mag_ctrl3_reg_address, ~MAG_CTRL3_REG::OCL, MAG_CTRL3_REG::OCL,
    set_state_mag(noop_state);
}

#pragma endregion

} // namespace MC6470

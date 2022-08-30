// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#ifndef INCLUDE_MC6470_ACCEL_HPP_
#define INCLUDE_MC6470_ACCEL_HPP_

#include <queue>
#include <utility>

#include "i2c_state_machine.hpp"
#include "mc6470_math.hpp"
#include "mc6470_registers_accel.hpp"

namespace MC6470 {

#define cast_state_acc(x) reinterpret_cast<state_machine_state>(&AccMC6470::x)
#define set_state_acc(x) this->current_state_function = cast_state_acc(x)

class AccMC6470 : public I2CStateMachine {
public:
    AccMC6470(int a5_state, TwoWire* _wire);

    void set_address_from_a5(int a5_state);

    void stop();

#pragma region GENERIC STATES

private:
    std::queue<state_machine_state> generic_state_queue; // note: must end in non-generic state

    void ensure_standby_mode();
    void check_standby_and_otp();

    void (*on_success_callback)() = 0;
    void call_success_callback();
#pragma endregion
#pragma region SET RESOLUTION AND RANGE

public:
    void set_res_and_range(uint8_t target_res, uint8_t _target_range, void (*callback_on_success)() = nullptr);

    uint8_t get_res_and_range();

private:
    uint8_t target_range = 0;
    uint8_t target_resolution = 0;

    void set_res_and_range_state();
#pragma endregion

// todo: These registers did nothing on one chip. Need to test others.
#pragma region SET GAINS AND OFFSETS

public:
    struct GainAndOffset {
        int16_t x_offset;
        int16_t y_offset;
        int16_t z_offset;
        float x_gain;
        float y_gain;
        float z_gain;
        uint16_t x_gain_chip;
        uint16_t y_gain_chip;
        uint16_t z_gain_chip;
    } target = {};

    void set_offset_and_gain(int16_t off_x, int16_t off_y, int16_t off_z, float gain_x, float gain_y, float gain_z,
        void (*callback_on_success)() = nullptr);

    GainAndOffset get_gain_and_offset();

private:
    void set_gain_and_offset_state();
#pragma endregion
#pragma region SET POLLING RATE

public:
    void set_rate(uint8_t _target_rate, void (*callback_on_success)() = nullptr);

    uint8_t get_rate();

private:
    uint8_t target_rate = ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ256;

    void set_rate_state();
#pragma endregion
#pragma region READ LOOP

public:
    template <int interrupt = -1>
    void start_accell_read_loop(
        void (*_acc_read_callback)(void*, int16_t, int16_t, int16_t) = nullptr, void* context = nullptr)
    {
        acc_read_callback = _acc_read_callback;
        acc_read_context = context;

        if constexpr (interrupt != -1) {
            interrupt_pin = interrupt;
            set_state_acc(ensure_standby_mode);
            generic_state_queue.push(cast_state_acc(ensure_inta_on_new_data));
            generic_state_queue.push(cast_state_acc(ensure_wake_mode));
            generic_state_queue.push(cast_state_acc(check_wake_and_otp));
            generic_state_queue.push(cast_state_acc(check_acq_int));
            run_state_on_interrupt<interrupt>(cast_state_acc(read_acc_xyz), RISING);
        } else {
            set_state_acc(ensure_wake_mode);
            generic_state_queue.push(cast_state_acc(ensure_wake_mode));
            generic_state_queue.push(cast_state_acc(check_wake_and_otp));
            generic_state_queue.push(cast_state_acc(check_acq_int));
        }
    }

    void set_offset(std::array<float, 3> new_offset);

    const std::array<float, 3>& get_offset() const&;

    void remove_offset();

    void set_xform(std::array<std::array<float, 3>, 3> new_xform);

    const std::array<std::array<float, 3>, 3>& get_xform() const&;

    void remove_xform();

private:
    int interrupt_pin = -1;
    bool has_offset = false;
    std::array<float, 3> offset_xyz = { 0, 0, 0 };
    bool has_xform = false;
    std::array<std::array<float, 3>, 3> lin_xform = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

    void ensure_inta_on_new_data();

    void ensure_wake_mode();

    void check_wake_and_otp();

    unsigned int mag_read_loop_wait_ms = 0;

    void check_acq_int();

    void read_acc_xyz();

    void (*acc_read_callback)(void*, int16_t, int16_t, int16_t) = nullptr;
    void* acc_read_context;
#pragma endregion

private:
    // todo: put command queue here?
    uint8_t address_accel;
};
} // namespace MC6470

#endif // INCLUDE_MC6470_ACCEL_HPP_

// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#ifndef INCLUDE_MC6470_MAG_HPP_
#define INCLUDE_MC6470_MAG_HPP_

#include <queue>
#include <utility>

#include "i2c_state_machine.hpp"
#include "mc6470_math.hpp"
#include "mc6470_registers_mag.hpp"

namespace MC6470 {
#define cast_state_mag(x) reinterpret_cast<state_machine_state>(&MagMC6470::x)
#define set_state_mag(x) this->current_state_function = cast_state_mag(x)

class MagMC6470 : public I2CStateMachine {
public:
    explicit MagMC6470(TwoWire* _wire);

    void (*on_success_callback)() = 0;
    void call_success_callback();

    // afaik, unlike the accelerometer, the magnetometer allows reading/writing bytes, except specific bits on
    // ctrl3, in any state, so the functions to set registers don't need to be state machines here.

    void set_dynamic_range(uint8_t range);

    void stop();

#pragma region GENERIC STATES

private:
    std::queue<state_machine_state> generic_state_queue; // note: must end in non-generic state

    void ensure_pc_generic_state();
    void ensure_standby_generic_state();
    void ensure_fs_generic_state();
    void ensure_fs_normal_generic_state();
    void ensure_ocl_generic_state();
    void ensure_ocl_normal_generic_state();
    void ensure_frc_generic_state();
#pragma endregion
#pragma region ASYNC READ LOOP

public:
    template <int interrupt = -1>
    void start_async_magnetometer_read(uint8_t rate = MAG_CTRL1_REG::ODR_HZ100,
        void (*_mag_read_callback)(void*, int16_t, int16_t, int16_t) = nullptr, void* context = nullptr)
    {
        // todo: ping drdy time could be modified as well, on top of loop wait. Currently ping drdy time is set to 2 ms.
        // todo: check replacing ensure_not_frc with ping_until_change of 1ms and check if it hangs
        // todo: raise error if _mag_read_callback is actually nullptr
        // todo: add option to replace ensure pc and ensure fs with in-memory checks for slight speedup? ...is it worth
        // it though? may be faster than the max 100hz. not tested.
        async_mag_rate = rate;
        async_reading = true;
        mag_read_loop_wait_ms = (unsigned int)(1000 / mag_get_rate_hz[rate] / 2);
        if (_mag_read_callback != nullptr)
            mag_read_callback = _mag_read_callback;
        mag_read_context = context;
        set_state_mag(begin_async_mag_state);
        generic_state_queue.push(cast_state_mag(set_intm_on_state));
        generic_state_queue.push(cast_state_mag(ensure_fs_normal_generic_state));

        if constexpr (interrupt != -1) {
            interrupt_pin = interrupt;
            generic_state_queue.push(cast_state_mag(noop_state));
            run_state_on_interrupt<interrupt>(cast_state_mag(finally_read_mag_state), RISING);
        } else {
            generic_state_queue.push(cast_state_mag(check_drdy_then_read_mag_state));
        }
    }

private:
    bool async_reading = true;

    void set_intm_on_state();
    void begin_async_mag_state();

    uint8_t async_mag_rate = MAG_CTRL1_REG::ODR_HZ100;
#pragma endregion
#pragma region FORCED READ LOOP

public:
    template <int interrupt = -1>
    void start_forced_magnetometer_read(uint32_t loop_wait = 0,
        void (*_mag_read_callback)(void*, int16_t, int16_t, int16_t) = nullptr, void* context = nullptr)
    {
        // todo: ping drdy time could be modified as well, on top of loop wait. Currently ping drdy time is set to 2 ms.
        // todo: check replacing ensure_not_frc with ping_until_change of 1ms and check if it hangs
        // todo: raise error if _mag_read_callback is actually nullptr
        // todo: add option to replace ensure pc and ensure fs with in-memory checks for slight speedup? ...is it worth
        // it though? may be faster than the max 100hz. not tested.
        async_reading = false;
        mag_read_loop_wait_ms = loop_wait;
        if (_mag_read_callback != nullptr)
            mag_read_callback = _mag_read_callback;
        mag_read_context = context;

        set_state_mag(ensure_pc_generic_state);
        generic_state_queue.push(cast_state_mag(ensure_fs_generic_state));
        generic_state_queue.push(cast_state_mag(ensure_frc_generic_state));
        generic_state_queue.push(cast_state_mag(ensure_not_frc_and_read_mag_state));

        if constexpr (interrupt != -1) {
            interrupt_pin = interrupt;
            run_state_on_interrupt<interrupt>(finally_read_mag_state, RISING);
        }
    }

    void set_mag_read_callback(void (*_mag_read_callback)(void*, int16_t, int16_t, int16_t));
    void set_offset(std::array<float, 3> new_offset);
    const std::array<float, 3>& get_offset() const&;
    void remove_offset();
    void set_xform(std::array<std::array<float, 3>, 3> new_xform);
    const std::array<std::array<float, 3>, 3>& get_xform() const&;
    void remove_xform();

private:
    bool has_offset = false;
    std::array<float, 3> offset_xyz = { 0, 0, 0 };
    bool has_xform = false;
    std::array<std::array<float, 3>, 3> lin_xform = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
    int interrupt_pin = -1;

    void ensure_not_frc_and_read_mag_state();
    void check_drdy_then_read_mag_state();
    void finally_read_mag_state();

    uint8_t mag_bits = 15; // todo: read this on startup to get accurate mag values

    void (*mag_read_callback)(void*, int16_t, int16_t, int16_t) = nullptr;
    void* mag_read_context = nullptr;
    uint32_t mag_read_loop_wait_ms = 10;
#pragma endregion
#pragma region TEMP READ STATE

public:
    void start_temp_read(void (*_temp_read_callback)(int8_t) = nullptr)
    {
        // todo: ping drdy time could be modified as well, on top of loop wait. Currently ping drdy time is set to 2 ms.
        // todo: check replacing ensure_not_frc with ping_until_change of 1ms and check if it hangs
        // todo: raise error if _temp_read_callback is actually nullptr
        // todo: add option to replace ensure pc and ensure fs with in-memory checks for slight speedup? ...is it worth
        // it though? may be faster than the max 100hz. not tested.
        temp_read_callback = _temp_read_callback;
        set_state_mag(ensure_pc_generic_state);
        // todo: do fs and tcs in one so it automatically goes back after measurement
        generic_state_queue.push(cast_state_mag(ensure_fs_generic_state));
        generic_state_queue.push(cast_state_mag(ensure_tcs_generic_state));
        generic_state_queue.push(cast_state_mag(finally_read_temp_state));
    }

private:
    void (*temp_read_callback)(int8_t) = nullptr;

    void ensure_tcs_generic_state();
    void finally_read_temp_state();
#pragma endregion
#pragma region SET MAG OFFSET

public:
    void set_mag_offset(int16_t x, int16_t y, int16_t z);

    std::auto_ptr<std::array<int16_t, 3>> get_mag_offsets();

private:
    int16_t target_mag_offset_x = 0;
    int16_t target_mag_offset_y = 0;
    int16_t target_mag_offset_z = 0;

    void finally_set_offset_state();
#pragma endregion
};
} // namespace MC6470

#endif // INCLUDE_MC6470_MAG_HPP_

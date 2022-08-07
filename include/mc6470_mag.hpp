#ifndef __MC6470_MAG_HPP
#define __MC6470_MAG_HPP

#include "mc6470_registers_mag.hpp"
#include "i2c_state_machine.hpp"
#include "mc6470_math.hpp"
#include <queue>

namespace MC6470
{
#define cast_state_mag(x) reinterpret_cast<state_machine_state>(&MagMC6470::x)
#define set_state_mag(x) this->current_state_function = cast_state_mag(x)

    class MagMC6470 : public I2CStateMachine
    {
    public:
        MagMC6470(TwoWire &_wire) : I2CStateMachine(_wire)
        {
            func_print;
        }

        void noop_state() {}

        void (*on_success_callback)();
        void call_success_callback()
        {
            func_print;
            this->on_success_callback();
            set_state_mag(noop_state);
        }

        // afaik, unlike the accelerometer, the magnetometer allows reading/writing bytes, except specific bits on ctrl3, in any state
        //  so, the functions to set registers don't need to be state machines here.

        void set_dynamic_range(uint8_t range)
        {
            write_byte_with_mask(address_mag, mag_range_reg_address, range, MAG_RANGE_REG::RANGE_15BIT);
        }

        void stop()
        {
            set_state_mag(noop_state);
        }

#pragma region G GATHER
    public:
        float mag_to_earth_local = 0;
        // note: earth local gauss can be converted into gauss or tesla given GPS coordinates elg*gps_gauss
        //   http://www.geomag.bgs.ac.uk/data_service/models_compass/wmm_calc.html
        // Use that, or a known magnet and distance, to determine mag_to_gauss
        // this could also be used to determine the offset to true north
        float mag_to_earth_local_variance = 0;

        void start_el_gather()
        {
            is_gathering_el = true;
        }

        void stop_el_gather()
        {
            is_gathering_el = false;
        }

        void reset_el_gather()
        {
            gather_iel = 0;
        }

    private:
        void gather_mag_earth_local(int16_t x, int16_t y, int16_t z)
        {
            // must be added to grab_accel_callback or fusion_update_callback
            if (gather_iel >= max_gather_i)
            {
                return;
            }
            if (gather_iel == 0)
            {
                mag_to_earth_local = 0;
            }
            gather_iel += 1;
            auto tmp_inv = 1.0 / TwoNorm<int16_t, float>({x, y, z});
            auto tmp_var = (mag_to_earth_local_variance * (gather_iel - 1) - tmp_inv);
            mag_to_earth_local_variance = tmp_var * tmp_var / gather_iel;
            mag_to_earth_local = (mag_to_earth_local * (gather_iel - 1) + tmp_inv) / gather_iel;
        }

        int gather_iel = 0;
        int gather_igauss = 0;

        int max_gather_i = 10000;

        bool is_gathering_el = false;

#pragma endregion
#pragma region GENERIC STATES
    private:
        std::queue<state_machine_state> generic_state_queue; // note: must end in non-generic state

        void ensure_pc_generic_state()
        {
            func_print;
            write_until_read_back(address_mag, mag_ctrl1_reg_address, MAG_CTRL1_REG::PC, MAG_CTRL1_REG::PC, generic_state_queue.front());
            generic_state_queue.pop();
        }

        void ensure_fs_generic_state()
        {
            func_print;
            write_until_read_back(address_mag, mag_ctrl1_reg_address, MAG_CTRL1_REG::FS, MAG_CTRL1_REG::FS, generic_state_queue.front());
            generic_state_queue.pop();
        }

        void ensure_frc_generic_state()
        {
            func_print;
            write_until_read_back(address_mag, mag_ctrl3_reg_address, MAG_CTRL3_REG::FORCE, MAG_CTRL3_REG::FORCE, generic_state_queue.front());
            generic_state_queue.pop();
        }
#pragma endregion
#pragma region FORCED READ LOOP
    public:
        void start_forced_magnetometer_read(unsigned long loop_wait = 0, void (*_mag_read_callback)(int16_t, int16_t, int16_t) = nullptr)
        {
            func_print;
            // todo: ping drdy time could be modified as well, on top of loop wait. Currently ping drdy time is set to 2 ms.
            // todo: check replacing ensure_not_frc with ping_until_change of 1ms and check if it hangs
            // todo: raise error if _mag_read_callback is actually nullptr
            // todo: add option to replace ensure pc and ensure fs with in-memory checks for slight speedup? ...is it worth it though?
            // may be faster than the max 100hz. not tested.
            mag_read_loop_wait_ms = loop_wait;
            if (_mag_read_callback != nullptr)
                mag_read_callback = _mag_read_callback;
            set_state_mag(ensure_pc_generic_state);
            generic_state_queue.push(cast_state_mag(ensure_fs_generic_state));
            generic_state_queue.push(cast_state_mag(ensure_frc_generic_state));
            generic_state_queue.push(cast_state_mag(ensure_not_frc_and_read_mag_state));
        }

        void set_mag_read_callback(void (*_mag_read_callback)(int16_t, int16_t, int16_t))
        {
            mag_read_callback = _mag_read_callback;
        }

    private:
        void ensure_not_frc_and_read_mag_state()
        {
            func_print;
            write_until_read_back(address_mag, mag_ctrl3_reg_address, ~MAG_CTRL3_REG::FORCE, MAG_CTRL3_REG::FORCE, cast_state_mag(check_drdy_then_read_mag_state));
        }

        void check_drdy_then_read_mag_state()
        {
            func_print;
            ping_until_register_change(address_mag, mag_status_reg_address, MAG_STATUS_REG::DRDY, MAG_STATUS_REG::DRDY, cast_state_mag(finally_read_mag_state), 2);
        }

        void finally_read_mag_state()
        {
            func_print;
            // todo: turn this into a func, read individual bytes back, and build up mag xyz
            int16_t x = read_2byte(address_mag, 0x10);
            int16_t y = read_2byte(address_mag, 0x12);
            int16_t z = read_2byte(address_mag, 0x14);

            if (is_gathering_el)
                gather_mag_earth_local(x, y, z);

            mag_read_callback(x, y, z);

            if (mag_read_loop_wait_ms)
                wait(mag_read_loop_wait_ms, cast_state_mag(ensure_frc_generic_state));
            else
                set_state_mag(ensure_frc_generic_state);
            generic_state_queue.push(cast_state_mag(ensure_not_frc_and_read_mag_state));
        }

        void (*mag_read_callback)(int16_t, int16_t, int16_t) = nullptr;
        unsigned long mag_read_loop_wait_ms = 10;
#pragma endregion
#pragma region TEMP READ STATE
    public:
        void start_temp_read(void (*_temp_read_callback)(int8_t) = nullptr)
        {
            func_print;
            // todo: ping drdy time could be modified as well, on top of loop wait. Currently ping drdy time is set to 2 ms.
            // todo: check replacing ensure_not_frc with ping_until_change of 1ms and check if it hangs
            // todo: raise error if _temp_read_callback is actually nullptr
            // todo: add option to replace ensure pc and ensure fs with in-memory checks for slight speedup? ...is it worth it though?
            // may be faster than the max 100hz. not tested.
            temp_read_callback = _temp_read_callback;
            set_state_mag(ensure_pc_generic_state);
            // todo: do fs and tcs in one so it automatically goes back after measurement
            generic_state_queue.push(cast_state_mag(ensure_fs_generic_state));
            generic_state_queue.push(cast_state_mag(ensure_tcs_generic_state));
            generic_state_queue.push(cast_state_mag(finally_read_temp_state));
        }

    private:
        void (*temp_read_callback)(int8_t) = nullptr;

        void ensure_tcs_generic_state()
        {
            func_print;
            write_byte_with_mask(address_mag, mag_ctrl3_reg_address, MAG_CTRL3_REG::TCS, MAG_CTRL3_REG::TCS);
            ping_until_register_change(address_mag, mag_ctrl3_reg_address, ~MAG_CTRL3_REG::TCS, MAG_CTRL3_REG::TCS, generic_state_queue.front(), 2);
            generic_state_queue.pop();
        }

        void finally_read_temp_state()
        {
            func_print;
            // todo: turn this into a func, read individual bytes back, and build up mag xyz
            int8_t t = read_byte(address_mag, 0x31);

            set_state_mag(noop_state);

            temp_read_callback(t);
        }
#pragma endregion
#pragma region SET MAG OFFSET
    public:
        void set_mag_offset(int16_t x, int16_t y, int16_t z)
        {
            func_print;
            // todo: ping drdy time could be modified as well, on top of loop wait. Currently ping drdy time is set to 2 ms.
            // todo: check replacing ensure_not_frc with ping_until_change of 1ms and check if it hangs
            // todo: add option to replace ensure pc and ensure fs with in-memory checks for slight speedup? ...is it worth it though?
            // may be faster than the max 100hz. not tested.
            target_mag_offset_x = x;
            target_mag_offset_y = y;
            target_mag_offset_z = z;

            set_state_mag(ensure_pc_generic_state);
            // todo: do fs and tcs in one so it automatically goes back after measurement
            generic_state_queue.push(cast_state_mag(ensure_fs_generic_state));
            generic_state_queue.push(cast_state_mag(finally_set_offset_state));
        }

        std::auto_ptr<std::array<int16_t, 3>> get_mag_offsets()
        {
            int16_t *c = new int16_t[3];

            c[0] = read_2byte(address_mag, 0x20);
            c[1] = read_2byte(address_mag, 0x22);
            c[2] = read_2byte(address_mag, 0x24);

            std::array<int16_t, 3> *A = reinterpret_cast<std::array<int16_t, 3> *>(c);
            std::auto_ptr<std::array<int16_t, 3>> a_ptr(A);
            return a_ptr;
        }

    private:
        int16_t target_mag_offset_x;
        int16_t target_mag_offset_y;
        int16_t target_mag_offset_z;

        void finally_set_offset_state()
        {
            func_print;

            write_byte_with_mask(address_mag, mag_ctrl3_reg_address, MAG_CTRL3_REG::OCL, MAG_CTRL3_REG::OCL);
            // if the mag had offsets already, what we measured included those.

            auto offsets = get_mag_offsets();
            write_2byte(address_mag, 0x20, (*offsets)[0] + target_mag_offset_x);
            write_2byte(address_mag, 0x22, (*offsets)[1] + target_mag_offset_y);
            write_2byte(address_mag, 0x24, (*offsets)[2] + target_mag_offset_z);
            write_until_read_back(address_mag, mag_ctrl3_reg_address, ~MAG_CTRL3_REG::OCL, MAG_CTRL3_REG::OCL, cast_state_mag(noop_state));
        }
#pragma endregion
    private:
        // todo: move these bools into a uint8
        // bool ready = true;
        // uint8_t mag_state;
    };
}

#endif
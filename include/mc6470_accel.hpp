#ifndef __MC6470_ACCEL_HPP
#define __MC6470_ACCEL_HPP

#include "mc6470_registers_accel.hpp"
#include "i2c_state_machine.hpp"
#include "mc6470_math.hpp"
#include <queue>

namespace MC6470
{

#define cast_state_acc(x) reinterpret_cast<state_machine_state>(&AccMC6470::x)
#define set_state_acc(x) this->current_state_function = cast_state_acc(x)

    class AccMC6470 : public I2CStateMachine
    {
    public:
        AccMC6470(int a5_state, TwoWire &_wire) : I2CStateMachine(_wire)
        {
            func_print;
            set_address_from_a5(a5_state);
        }

        void set_address_from_a5(int a5_state)
        {
            func_print;
            // Adding this as a function in case, for some god forsaken reason, someone wants to switch addresses while the device is running
            if (a5_state)
            {
                this->address_accel = address_accel_a5vcc;
            }
            else
            {
                this->address_accel = address_accel_a5gnd;
            }
        }

        void stop()
        {
            set_state_acc(noop_state);
        }
#pragma region G GATHER

    public:
        float acc_to_g = 0;
        float acc_to_g_variance = 0;

        void start_g_gather()
        {
            is_gathering_g = true;
        }

        void stop_g_gather()
        {
            is_gathering_g = false;
        }

        void reset_g_gather()
        {
            gather_ig = 0;
        }

        template <class T, class R = T>
        std::auto_ptr<std::array<R, 3>> convert_acc_to_g(const std::array<T, 3> &a)
        {
            R *c = new R[3];
            c[0] = (R)a[0] * acc_to_g;
            c[1] = (R)a[1] * acc_to_g;
            c[2] = (R)a[2] * acc_to_g;

            std::array<R, 3> &A = reinterpret_cast<std::array<R, 3> &>(c);
            std::auto_ptr<std::array<R, 3>> a_ptr(&A);
            return a_ptr;
        }

    private:
        void gather_g(int16_t x, int16_t y, int16_t z)
        {
            // must be added to grab_accel_callback or fusion_update_callback
            if (gather_ig >= max_gather_i)
            {
                return;
            }
            if (gather_ig == 0)
            {
                acc_to_g = 0;
            }
            gather_ig += 1;
            auto tmp_inv = 1.0 / TwoNorm<int16_t, float>({x, y, z});
            auto tmp_var = (acc_to_g_variance * (gather_ig - 1) - tmp_inv);
            acc_to_g_variance = tmp_var * tmp_var / gather_ig;
            acc_to_g = (acc_to_g * (gather_ig - 1) + tmp_inv) / gather_ig;
        }

        bool is_gathering_g = false;
        int gather_ig = 0;
        int max_gather_i = 10000;

#pragma endregion
#pragma region GENERIC STATES
    private:
        std::queue<state_machine_state> generic_state_queue; // note: must end in non-generic state

        void noop_state() {}

        void ensure_standby_mode()
        {
            func_print;
            write_until_read_back(address_accel, acc_mode_reg_address, ACC_MODE_REG::OPCON_STANDBY, ACC_MODE_REG::OPCON_MASK, generic_state_queue.front());
            generic_state_queue.pop();
        }

        void check_standby_and_otp()
        {
            func_print;
            uint8_t full_val = ACC_OPSTAT_REG::OPSTAT_STANDBY | ACC_OPSTAT_REG::OTPA_IDLE;
            uint8_t full_mask = ACC_OPSTAT_REG::OPSTAT_MASK | ACC_OPSTAT_REG::OTPA_MASK;
            ping_until_register_change(address_accel, acc_opstat_reg_address, full_val, full_mask, generic_state_queue.front(), 2);
            generic_state_queue.pop();
        }

        void (*on_success_callback)();
        void call_success_callback()
        {
            func_print;
            this->on_success_callback();
            set_state_acc(noop_state);
        }
#pragma endregion
#pragma region SET RESOLUTION AND RANGE
    public:
        void set_res_and_range(uint8_t target_res, uint8_t _target_range, void (*callback_on_success)())
        {
            on_success_callback = callback_on_success;
            target_resolution = target_res;
            target_range = _target_range;
            set_state_acc(ensure_standby_mode);
            generic_state_queue.push(cast_state_acc(check_standby_and_otp));
            generic_state_queue.push(cast_state_acc(set_res_and_range_state));
        }

        uint8_t get_res_and_range()
        {
            uint8_t res_range_byte = read_byte(address_accel, acc_outcfg_reg_address);
            return res_range_byte;
        }

    private:
        uint8_t target_range = 0;
        uint8_t target_resolution = 0;

        void set_res_and_range_state()
        {
            func_print;
            uint8_t full_val = target_range | target_resolution;
            uint8_t full_mask = ACC_OUTCFG_REG::RANGE_MASK | ACC_OUTCFG_REG::RES_MASK;
            write_until_read_back(address_accel, acc_outcfg_reg_address, full_val, full_mask, cast_state_acc(call_success_callback), 1000, cast_state_acc(noop_state));
        }
#pragma endregion
#pragma region SET GAINS AND OFFSETS
    public:
        struct GainAndOffset
        {
            int16_t x_offset;
            int16_t y_offset;
            int16_t z_offset;
            float x_gain;
            float y_gain;
            float z_gain;
            uint16_t x_gain_chip;
            uint16_t y_gain_chip;
            uint16_t z_gain_chip;
        } target;

        void set_offset_and_gain(
            int16_t off_x, int16_t off_y, int16_t off_z,
            float gain_x, float gain_y, float gain_z,
            void (*callback_on_success)())
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

        GainAndOffset get_gain_and_offset()
        {
            func_print;
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

    private:
        void set_gain_and_offset_state()
        {
            func_print;

            GainAndOffset to_send = get_gain_and_offset();
            to_send.x_offset = add_with_limits_signed<int16_t, 15>(to_send.x_offset, target.x_offset);
            to_send.y_offset = add_with_limits_signed<int16_t, 15>(to_send.y_offset, target.y_offset);
            to_send.z_offset = add_with_limits_signed<int16_t, 15>(to_send.z_offset, target.z_offset);
            to_send.x_gain_chip = multiply_with_limits_unsigned<uint16_t, 9, float>(to_send.x_gain_chip, target.x_gain);
            to_send.y_gain_chip = multiply_with_limits_unsigned<uint16_t, 9, float>(to_send.y_gain_chip, target.y_gain);
            to_send.z_gain_chip = multiply_with_limits_unsigned<uint16_t, 9, float>(to_send.z_gain_chip, target.z_gain);

            write_byte(address_accel, 0x20, (uint8_t)to_send.x_offset);
            write_byte(address_accel, 0x21, ((uint8_t)(((uint16_t)to_send.x_offset) >> 8) & 0b01111111) | to_send.x_gain_chip >> 8);
            write_byte(address_accel, 0x22, (uint8_t)to_send.y_offset);
            write_byte(address_accel, 0x23, ((uint8_t)(((uint16_t)to_send.y_offset) >> 8) & 0b01111111) | to_send.y_gain_chip >> 8);
            write_byte(address_accel, 0x24, (uint8_t)to_send.z_offset);
            write_byte(address_accel, 0x25, ((uint8_t)(((uint16_t)to_send.z_offset) >> 8) & 0b01111111) | to_send.z_gain_chip >> 8);
            write_byte(address_accel, 0x26, (uint8_t)to_send.x_gain_chip);
            write_byte(address_accel, 0x27, (uint8_t)to_send.y_gain_chip);
            write_byte(address_accel, 0x28, (uint8_t)to_send.z_gain_chip);

            set_state_acc(noop_state);

            this->on_success_callback();
        }
#pragma endregion
#pragma region SET POLLING RATE
    public:
        void set_rate(uint8_t _target_rate, void (*callback_on_success)())
        {
            on_success_callback = callback_on_success;
            target_rate = _target_rate;
            set_state_acc(ensure_standby_mode);
            generic_state_queue.push(cast_state_acc(check_standby_and_otp));
            generic_state_queue.push(cast_state_acc(set_rate_state));
        }

        uint8_t get_rate()
        {
            func_print;
            uint8_t rate = read_byte(address_accel, acc_srtf_reg_address);
            return rate;
        }

    private:
        uint8_t target_rate = ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ256;

        void set_rate_state()
        {
            func_print;
            write_until_read_back(address_accel, acc_srtf_reg_address, target_rate, ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_MASK, cast_state_acc(call_success_callback), 1000, cast_state_acc(noop_state));
        }
#pragma endregion
#pragma region READ LOOP
    public:
        void start_accell_read_loop(void (*_acc_read_callback)(int16_t, int16_t, int16_t) = nullptr)
        {
            acc_read_callback = _acc_read_callback;
            set_state_acc(ensure_wake_mode);
            generic_state_queue.push(cast_state_acc(check_wake_and_otp));
            generic_state_queue.push(cast_state_acc(check_acq_int));
        }

    private:
        void ensure_wake_mode()
        {
            func_print;
            write_until_read_back(address_accel, acc_mode_reg_address, ACC_MODE_REG::OPCON_WAKE, ACC_MODE_REG::OPCON_MASK, generic_state_queue.front());
            generic_state_queue.pop();
        }

        void check_wake_and_otp()
        {
            func_print;
            uint8_t full_val = ACC_OPSTAT_REG::OPSTAT_WAKE | ACC_OPSTAT_REG::OTPA_IDLE;
            uint8_t full_mask = ACC_OPSTAT_REG::OPSTAT_MASK | ACC_OPSTAT_REG::OTPA_MASK;
            ping_until_register_change(address_accel, acc_opstat_reg_address, full_val, full_mask, generic_state_queue.front(), 2);
            generic_state_queue.pop();
        }

        unsigned int mag_read_loop_wait_ms = 0;

        void check_acq_int()
        {
            func_print;
            mag_read_loop_wait_ms = (unsigned int)((1000 / acc_get_rate_hz[target_rate]) / 2);
            ping_until_register_change(address_accel, acc_sr_reg_address, ACC_SR_REG::ACQ_INT, ACC_SR_REG::ACQ_INT, cast_state_acc(read_acc_xyz), mag_read_loop_wait_ms);
        }

        void read_acc_xyz()
        {
            func_print;
            uint8_t *bytes = burst_read_bytes(address_accel, acc_xyz_reg_address, 6);

            int16_t x = 0 | bytes[0] | bytes[1] << 8;
            int16_t y = 0 | bytes[2] | bytes[3] << 8;
            int16_t z = 0 | bytes[3] | bytes[5] << 8;

            if (is_gathering_g)
                gather_g(x, y, z);

            acc_read_callback(x, y, z);

            set_state_acc(check_acq_int);

            free(bytes);
        }

        void (*acc_read_callback)(int16_t, int16_t, int16_t) = nullptr;
#pragma endregion
    private:
        // todo: put command queue here?
        uint8_t address_accel;
    };
}

#endif
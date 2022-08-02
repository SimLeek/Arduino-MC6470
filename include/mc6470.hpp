#ifndef __MC6470_HPP
#define __MC6470_HPP

#include <Arduino.h>
#include <Wire.h>
#include "i2c_state_machine.hpp"
#include <queue>

namespace MC6470
{

  const uint8_t address_accel_a5gnd = 0x4c;
  const uint8_t address_accel_a5vcc = 0x6c;
  const uint8_t address_mag = 0x0c;

  const uint8_t mag_whoami_address = 0x0f;
  const uint8_t mag_whoami_expected = 0x49;
  const int error_mag_whoami_mismatch = -1;

  const uint8_t acc_xyz_reg_address = 0x0D;

  typedef enum : uint8_t
  {
    ACQ_INT = 0b10000000,
  } ACC_SR_REG;
  const uint8_t acc_sr_reg_address = 0x03;

  typedef enum : uint8_t
  {
    RATE_HZ32 = 0b00000000,
    RATE_HZ16 = 0b00000001,
    RATE_HZ8 = 0b00000010,
    RATE_HZ4 = 0b00000011,
    RATE_HZ2 = 0b00000100,
    RATE_HZ1 = 0b00000101,
    RATE_HZ0P5 = 0b00000110,
    RATE_HZ0P25 = 0b00000111,
    RATE_HZ64 = 0b00001000,
    RATE_HZ128 = 0b00001001,
    RATE_HZ256 = 0b00001010,
    RATE_MASK = 0b00001111,
  } ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG;
  const uint8_t acc_srtf_reg_address = 0x08;

  std::map<uint8_t, float> acc_get_rate_hz{
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ32, 32),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ16, 16),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ8, 8),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ4, 4),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ2, 2),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ1, 1),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ0P5, 0.5),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ0P25, 0.25),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ64, 64),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ128, 128),
      std::make_pair(ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_HZ256, 256),
  };

  typedef enum : uint8_t
  {
    OPCON_STANDBY = 0b00000000,
    OPCON_WAKE = 0b00000001,
    OPCON_MASK = 0b00000011
  } ACC_MODE_REG;
  const uint8_t acc_mode_reg_address = 0x07;

  typedef enum : uint8_t
  {
    OPSTAT_STANDBY = 0b00000000,
    OPSTAT_WAKE = 0b00000001,
    OPSTAT_MASK = 0b00000011,

    OTPA_IDLE = 0b00000000,
    OTPA_ACTIVE = 0b10000000,
    OTPA_MASK = 0b10000000,
  } ACC_OPSTAT_REG;
  const uint8_t acc_opstat_reg_address = 0x04;

  typedef enum : uint8_t
  {
    RES_6BIT = 0b00000000,
    RES_7BIT = 0b00000001,
    RES_8BIT = 0b00000010,
    RES_10BIT = 0b00000011,
    RES_12BIT = 0b00000100,
    RES_14BIT = 0b00000101,
    RES_MASK = 0b00000111,

    RANGE_2G = 0b00000000,
    RANGE_4G = 0b00010000,
    RANGE_8G = 0b00100000,
    RANGE_16G = 0b00110000,
    RANGE_MASK = 0b11110000, // Hack: bit 7 must always be 0, so range now sets it to that
  } ACC_OUTCFG_REG;
  const uint8_t acc_outcfg_reg_address = 0x20;

  typedef enum : uint8_t
  {
    PC = 0b10000000,
    ODR_HZ0P5 = 0b00000000,
    ODR_HZ10 = 0b00001000,
    ODR_HZ20 = 0b00010000,
    ODR_HZ100 = 0b00011000,
    FS = 0b00000010,
  } MAG_CTRL1_REG;
  const uint8_t mag_ctrl1_reg_address = 0x1B;

  typedef enum : uint8_t
  {
    FORCE = 0b01000000,
    TCS = 0b00000100,
    OCL = 0b00001000
  } MAG_CTRL3_REG;
  const uint8_t mag_ctrl3_reg_address = 0x1D;

  typedef enum : uint8_t
  {
    DRDY = 0b01000000,
  } MAG_STATUS_REG;
  const uint8_t mag_status_reg_address = 0x18;

  typedef enum : uint8_t
  {
    RANGE_14BIT = 0b00000000,
    RANGE_15BIT = 0b00010000,
  } MAG_RANGE_REG;
  const uint8_t mag_range_reg_address = 0x1E;

  class AccMC6470 : public I2CStateMachine
  {
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

    std::queue<state_machine_state> generic_state_queue; // note: must end in non-generic state

    /********************
     GENERIC STATES
    *********************/
  private:
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
      set_state(noop_state);
    }

    /********************
     SET RES AND RANGE
    *********************/
  public:
    void set_res_and_range(uint8_t target_res, uint8_t _target_range, void (*callback_on_success)())
    {
      on_success_callback = callback_on_success;
      target_resolution = target_res;
      target_range = _target_range;
      set_state(ensure_standby_mode);
      generic_state_queue.push(cast_state(check_standby_and_otp));
      generic_state_queue.push(cast_state(set_res_and_range_state));
    }

  private:
    uint8_t target_range = 0;
    uint8_t target_resolution = 0;

    void (*on_success_callback)();

    void set_res_and_range_state()
    {
      func_print;
      uint8_t full_val = target_range | target_resolution;
      uint8_t full_mask = ACC_OUTCFG_REG::RANGE_MASK | ACC_OUTCFG_REG::RES_MASK;
      write_until_read_back(address_accel, acc_outcfg_reg_address, full_val, full_mask, cast_state(call_success_callback), 1000, cast_state(noop_state));
    }

    /********************
     SET GAINS AND OFFSETS
    *********************/
  public:
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
      set_state(ensure_standby_mode);
      generic_state_queue.push(cast_state(check_standby_and_otp));
      generic_state_queue.push(cast_state(set_gain_and_offset_state));
    }

    struct GainAndOffset{
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

  private:
    
    GainAndOffset get_gain_and_offset(){
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

      current.x_offset = ((uint16_t)0 | xoff_lsb | ((uint16_t)(xoff_msb&0b01111111))<<8)<<1;
      current.y_offset = ((uint16_t)0 | yoff_lsb | ((uint16_t)(yoff_msb&0b01111111))<<8)<<1;
      current.z_offset = ((uint16_t)0 | zoff_lsb | ((uint16_t)(zoff_msb&0b01111111))<<8)<<1;

      current.x_gain = (uint16_t)0 | xgain | ((uint16_t)(xoff_msb&0b10000000))<<1;
      current.y_gain = (uint16_t)0 | ygain | ((uint16_t)(yoff_msb&0b10000000))<<1;
      current.z_gain = (uint16_t)0 | zgain | ((uint16_t)(zoff_msb&0b10000000))<<1;

      return current;
    }

    template<class T>
    T unsigned_overflow_limit(T x, size_t max_bits){
      T max_val = pow(2, max_bits)-1
      if (x > max_val){
        return max_val;
      }else{
        return x;
      }
    }

    void set_res_and_range_state()
    {
      func_print;
      uint8_t full_val = target_range | target_resolution;
      uint8_t full_mask = ACC_OUTCFG_REG::RANGE_MASK | ACC_OUTCFG_REG::RES_MASK;
      write_until_read_back(address_accel, acc_outcfg_reg_address, full_val, full_mask, cast_state(call_success_callback), 1000, cast_state(noop_state));
    }

    void set_res_and_range_state()
    {
      func_print;
      uint8_t full_val = target_range | target_resolution;
      uint8_t full_mask = ACC_OUTCFG_REG::RANGE_MASK | ACC_OUTCFG_REG::RES_MASK;
      write_until_read_back(address_accel, acc_outcfg_reg_address, full_val, full_mask, cast_state(call_success_callback), 1000, cast_state(noop_state));
    }

    /********************
     SET POLLING RATE
    *********************/
  public:
    void set_polling_rate(uint8_t target_rate, void (*callback_on_success)())
    {
      on_success_callback = callback_on_success;
      target_polling_rate = target_rate;
      set_state(ensure_standby_mode);
      generic_state_queue.push(cast_state(check_standby_and_otp));
      generic_state_queue.push(cast_state(set_rate));
    }

  private:
    uint8_t target_polling_rate = 0;

    void set_rate()
    {
      func_print;
      write_until_read_back(address_accel, acc_srtf_reg_address, target_polling_rate, ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_MASK, cast_state(call_success_callback), 1000, cast_state(noop_state));
    }

    /********************
     SET OFFSETS
    *********************/
  public:
    void set_polling_rate(uint8_t target_rate, void (*callback_on_success)())
    {
      on_success_callback = callback_on_success;
      target_polling_rate = target_rate;
      set_state(ensure_standby_mode);
      generic_state_queue.push(cast_state(check_standby_and_otp));
      generic_state_queue.push(cast_state(set_rate));
    }

  private:
    uint8_t target_polling_rate = 0;

    void set_rate()
    {
      func_print;
      write_until_read_back(address_accel, acc_srtf_reg_address, target_polling_rate, ACC_SAMPLE_RATE_AND_TAP_FEATURE_REG::RATE_MASK, cast_state(call_success_callback), 1000, cast_state(noop_state));
    }

    /********************
     READ LOOP
    *********************/
  public:
    void start_accell_read_loop(void (*_acc_read_callback)(int16_t, int16_t, int16_t) = nullptr)
    {
      acc_read_callback = _acc_read_callback;
      set_state(ensure_wake_mode);
      generic_state_queue.push(cast_state(check_wake_and_otp));
      generic_state_queue.push(cast_state(check_acq_int));
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
      mag_read_loop_wait_ms = (unsigned int)((1000 / acc_get_rate_hz[target_polling_rate]) / 2);
      ping_until_register_change(address_accel, acc_sr_reg_address, ACC_SR_REG::ACQ_INT, ACC_SR_REG::ACQ_INT, cast_state(read_acc_xyz), mag_read_loop_wait_ms);
    }

    void read_acc_xyz()
    {
      func_print;
      uint8_t *bytes = burst_read_bytes(address_accel, acc_xyz_reg_address, 6);

      int16_t x = 0 | bytes[0] | bytes[1] << 8;
      int16_t y = 0 | bytes[2] | bytes[3] << 8;
      int16_t z = 0 | bytes[3] | bytes[5] << 8;

      acc_read_callback(x, y, z);

      set_state(check_acq_int);

      free(bytes);
    }

    void (*acc_read_callback)(int16_t, int16_t, int16_t) = nullptr;

  private:
    // todo: put command queue here?
    uint8_t address_accel;
  };

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
      set_state(noop_state);
    }

    // afaik, unlike the accelerometer, the magnetometer allows reading/writing bytes, except specific bits on ctrl3, in any state
    //  so, the functions to set registers don't need to be state machines here.

    void set_dynamic_range(uint8_t range)
    {
      write_byte_with_mask(address_mag, mag_range_reg_address, range, MAG_RANGE_REG::RANGE_15BIT);
    }

    /********************
     FORCED READ STATE
    *********************/
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
      mag_read_callback = _mag_read_callback;
      set_state(ensure_pc_generic_state);
      generic_state_queue.push(cast_state(ensure_fs_generic_state));
      generic_state_queue.push(cast_state(ensure_frc_generic_state));
      generic_state_queue.push(cast_state(ensure_not_frc_and_read_mag_state));
    }

    std::queue<state_machine_state> generic_state_queue; // note: must end in non-generic state

  private:
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

    void ensure_not_frc_and_read_mag_state()
    {
      func_print;
      write_until_read_back(address_mag, mag_ctrl3_reg_address, ~MAG_CTRL3_REG::FORCE, MAG_CTRL3_REG::FORCE, cast_state(check_drdy_then_read_mag_state));
    }

    void check_drdy_then_read_mag_state()
    {
      func_print;
      ping_until_register_change(address_mag, mag_status_reg_address, MAG_STATUS_REG::DRDY, MAG_STATUS_REG::DRDY, cast_state(finally_read_mag_state), 2);
    }

    void finally_read_mag_state()
    {
      func_print;
      // todo: turn this into a func, read individual bytes back, and build up mag xyz
      int16_t x = read_2byte(address_mag, 0x10);
      int16_t y = read_2byte(address_mag, 0x12);
      int16_t z = read_2byte(address_mag, 0x14);

      mag_read_callback(x, y, z);

      if (mag_read_loop_wait_ms)
        wait(mag_read_loop_wait_ms, cast_state(ensure_frc_generic_state));
      else
        set_state(ensure_frc_generic_state);
      generic_state_queue.push(cast_state(ensure_not_frc_and_read_mag_state));
    }

    void (*mag_read_callback)(int16_t, int16_t, int16_t) = nullptr;
    unsigned long mag_read_loop_wait_ms = 10;

    /********************
     TEMP READ STATE
    *********************/
  public:
    void start_temp_read(void (*_temp_read_callback)(int8_t) = nullptr)
    {
      func_print;
      // todo: ping drdy time could be modified as well, on top of loop wait. Currently ping drdy time is set to 2 ms.
      // todo: check replacing ensure_not_frc with ping_until_change of 1ms and check if it hangs
      // todo: raise error if _mag_read_callback is actually nullptr
      // todo: add option to replace ensure pc and ensure fs with in-memory checks for slight speedup? ...is it worth it though?
      // may be faster than the max 100hz. not tested.
      temp_read_callback = _temp_read_callback;
      set_state(ensure_pc_generic_state);
      // todo: do fs and tcs in one so it automatically goes back after measurement
      generic_state_queue.push(cast_state(ensure_fs_generic_state));
      generic_state_queue.push(cast_state(ensure_tcs_generic_state));
      generic_state_queue.push(cast_state(finally_read_temp_state));
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

      set_state(noop_state);

      temp_read_callback(t);
    }

    /********************
     MAG OFFSET STATE
    *********************/

  public:
    void set_mag_offset(int16_t x, int16_t y, int16_t z)
    {
      func_print;
      // todo: ping drdy time could be modified as well, on top of loop wait. Currently ping drdy time is set to 2 ms.
      // todo: check replacing ensure_not_frc with ping_until_change of 1ms and check if it hangs
      // todo: raise error if _mag_read_callback is actually nullptr
      // todo: add option to replace ensure pc and ensure fs with in-memory checks for slight speedup? ...is it worth it though?
      // may be faster than the max 100hz. not tested.
      target_mag_offset_x = x;
      target_mag_offset_y = y;
      target_mag_offset_z = z;

      set_state(ensure_pc_generic_state);
      // todo: do fs and tcs in one so it automatically goes back after measurement
      generic_state_queue.push(cast_state(ensure_fs_generic_state));
      generic_state_queue.push(cast_state(finally_set_offset_state));
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
      int16_t original_x = read_2byte(address_mag, 0x20);
      write_2byte(address_mag, 0x20, original_x + target_mag_offset_x);
      int16_t original_y = read_2byte(address_mag, 0x22);
      write_2byte(address_mag, 0x22, original_y + target_mag_offset_y);
      int16_t original_z = read_2byte(address_mag, 0x24);
      write_2byte(address_mag, 0x24, original_z + target_mag_offset_z);
      write_until_read_back(address_mag, mag_ctrl3_reg_address, ~MAG_CTRL3_REG::OCL, MAG_CTRL3_REG::OCL, cast_state(noop_state));
    }

  private:
    // todo: move these bools into a uint8
    bool ready = true;
    uint8_t mag_state;
  };

  /*
  class MC6470OLD : StateMachine{

  public:

    //void set_state_on_interrupt(int pin, )



    int check_whoami(){
      uint16_t rcvData;
      wire.requestFrom(address_mag, 1);
      if(wire.available())
      {
          rcvData = wire.read();
      }
      if (rcvData!=mag_whoami_expected){
          return -error_mag_whoami_mismatch;
      }
      // only the magnetometer has a whoami. the accel has a product code that can change.
      return 1;
    }

    byte read_byte(uint16_t i2c_address, uint16_t register_address){
      Wire.beginTransmission(i2c_address);
      Wire.write(register_address);
      Wire.endTransmission();
      uint16_t rcvData;
      wire.requestFrom(i2c_address, 1);
      if(wire.available())
      {
          rcvData = wire.read();
      }
      return rcvData;
    }

    void write_byte(uint16_t i2c_address, uint16_t register_address, uint16_t value){
      Wire.beginTransmission(i2c_address);
      Wire.write(register_address);
      Wire.write(value);
      Wire.endTransmission();
    }

    int mag_set_active(){
      uint16_t current_reg = read_byte(address_mag, 0x1B);
      write_byte(0b10000000&current_reg);
    }

    int mag_set_standby(){
      uint16_t current_reg = read_byte(address_mag, 0x1B);
      write_byte(0b01111111|current_reg);
    }

    int mag_set_data_rate(MAG_DATA_RATE dr){
      uint16_t current_reg = read_byte(address_mag, 0x1B);
      current_reg &= ~(0b11<<3);
      current_reg |= (dr<<3);
      write_byte(address_mag, 0x1B, current_reg);
    }

    int mag_set_force_state_off(){
      uint16_t current_reg = read_byte(address_mag, 0x1B);
      write_byte(0b11111101|current_reg);
    }

    int mag_set_force_state_on(){
      uint16_t current_reg = read_byte(address_mag, 0x1B);
      write_byte(0b00000010&current_reg);
    }

    int mag_set_high_speed_mode(){
      write_byte(address_mag, 0x1B, 0b10011010);
    }

  private:

  };*/

}

#endif
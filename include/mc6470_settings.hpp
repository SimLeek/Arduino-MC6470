// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#ifndef INCLUDE_MC6470_SETTINGS_HPP_
#define INCLUDE_MC6470_SETTINGS_HPP_

#include "mc6470_accel.hpp"
#include "mc6470_mag.hpp"
#include <vector>
#include <array>
# include <utility>

namespace MC6470 {

class MC6470Settings {
public:
    explicit MC6470Settings(int address = 0)
        : start_address(address)
    {
    }

    ~MC6470Settings();

    void add_mag_ardu_correction(std::array<std::array<float, 3>, 3>* mat, std::array<float, 3>* off);

    void add_mag_ardu_correction(std::array<float, 12>* mat);

    void add_acc_ardu_correction(std::array<std::array<float, 3>, 3>* mat, std::array<float, 3>* off);

    void add_acc_ardu_correction(std::array<float, 12>* mat);

    void add_imu_mag_correction(std::array<int16_t, 3>* off);

    void add_imu_acc_correction(std::array<float, 3>* scale, std::array<int16_t, 3>* off);

    void add_imu_acc_correction(std::array<uint8_t, 18>* full);

    void add_acc_res_range(uint8_t res_range_setting);

    void add_acc_rate(uint8_t rate);

    int write_to_eeprom();

    int read_from_eeprom();

    void apply(MC6470::MagMC6470* mag_machine = nullptr, MC6470::AccMC6470* acc_machine = nullptr);

private:
    typedef enum {
        MAG_ARDU_CORRECTION,
        ACC_ARDU_CORRECTION,
        MAG_IMU_CORRECTION,
        ACC_IMU_CORRECTION,
        ACC_RES_RANGE,
        ACC_RATE,
        END
    } MEMTYPE;

    void add_ardu_correction(std::array<std::array<float, 3>, 3>* mat, std::array<float, 3>* off, MEMTYPE the_type);

    std::vector<std::pair<MEMTYPE, void*>> mem;

    void del_mem_vector();

    int start_address = 0;
};
} // namespace MC6470

#endif // INCLUDE_MC6470_SETTINGS_HPP_

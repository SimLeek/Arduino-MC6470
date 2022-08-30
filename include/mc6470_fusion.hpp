// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#ifndef INCLUDE_MC6470_FUSION_HPP_
#define INCLUDE_MC6470_FUSION_HPP_

#include "mc6470_accel.hpp"
#include "mc6470_mag.hpp"
#include "mc6470_math.hpp"

namespace MC6470 {
void fuse_mag_callback(void* context, int16_t a, int16_t b, int16_t c);
void fuse_acc_callback(void* context, int16_t a, int16_t b, int16_t c);

class FuseMC6470 {
public:
    explicit FuseMC6470(MagMC6470* mag, AccMC6470* accel)
        : the_mag(mag)
        , the_acc(accel)
    {
    }

    template <int int1 = -1, int int2 = -1> void default_setup()
    {
        the_mag->start_async_magnetometer_read<int1>(MC6470::MAG_CTRL1_REG::ODR_HZ100, MC6470::fuse_mag_callback, this);
        the_acc->start_accell_read_loop<int2>(MC6470::fuse_acc_callback, this);

        if constexpr (int1 != -1)
            the_mag->update_until_done();
        if constexpr (int2 != -1)
            the_acc->update_until_done();
    }

    void update();

    std::auto_ptr<std::array<int32_t, 9>> get_orientation_matrix();

    void grab_accel_callback(int16_t x, int16_t y, int16_t z);

    void grab_mag_callback(int16_t x, int16_t y, int16_t z);

    void grab_temp_callback(int8_t t);

    void set_fusion_callback(void (*callback)(std::auto_ptr<std::array<int32_t, 9>>));

private:
    MagMC6470* the_mag;
    AccMC6470* the_acc;

    int16_t last_acc[3] = { 0 };
    int16_t last_mag[3] = { 0 };
    int32_t east[3] = { 0 };
    int8_t temp_in_celcius = 0;

    void (*fuse_callback)(std::auto_ptr<std::array<int32_t, 9>>) = nullptr;

    std::auto_ptr<std::array<int16_t, 3>> update_fusion();
};

void fuse_mag_callback(void* context, int16_t a, int16_t b, int16_t c);

void fuse_acc_callback(void* context, int16_t a, int16_t b, int16_t c);

} // namespace MC6470

#endif // INCLUDE_MC6470_FUSION_HPP_

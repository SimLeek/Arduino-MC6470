// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470_fusion.hpp"

namespace MC6470 {

void FuseMC6470::update()
{
    the_mag->update();
    the_acc->update();
}

std::auto_ptr<std::array<int32_t, 9>> FuseMC6470::get_orientation_matrix()
{
    // std::array<int32_t, 9> c();
    int32_t* c = new int32_t[9];

    c[0] = east[0];
    c[1] = east[1];
    c[2] = east[2];

    c[3] = last_mag[0];
    c[4] = last_mag[1];
    c[5] = last_mag[2];

    c[6] = last_acc[0];
    c[7] = last_acc[1];
    c[8] = last_acc[2];

    std::array<int32_t, 9>* A = reinterpret_cast<std::array<int32_t, 9>*>(c);
    std::auto_ptr<std::array<int32_t, 9>> a_ptr(A);
    return a_ptr;
}

void FuseMC6470::grab_accel_callback(int16_t x, int16_t y, int16_t z)
{
    last_acc[0] = x;
    last_acc[1] = y;
    last_acc[2] = z;
    update_fusion();
}

void FuseMC6470::grab_mag_callback(int16_t x, int16_t y, int16_t z)
{
    last_mag[0] = x;
    last_mag[1] = y;
    last_mag[2] = z;
    update_fusion();
}

void FuseMC6470::grab_temp_callback(int8_t t) { temp_in_celcius = t; }

void FuseMC6470::set_fusion_callback(void (*callback)(std::auto_ptr<std::array<int32_t, 9>>))
{
    fuse_callback = callback;
}

std::auto_ptr<std::array<int16_t, 3>> FuseMC6470::update_fusion()
{
    auto cross = CrossProduct(last_acc, last_mag);
    east[0] = (*cross)[0];
    east[1] = (*cross)[1];
    east[2] = (*cross)[2];

    if (fuse_callback != nullptr) {
        auto orient = get_orientation_matrix();

        fuse_callback(orient);
    }

    return cross;
}

void fuse_mag_callback(void* context, int16_t a, int16_t b, int16_t c)
{
    static_cast<FuseMC6470*>(context)->grab_mag_callback(a, b, c);
}

void fuse_acc_callback(void* context, int16_t a, int16_t b, int16_t c)
{
    static_cast<FuseMC6470*>(context)->grab_accel_callback(a, b, c);
}

} // namespace MC6470

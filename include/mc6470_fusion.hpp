#ifndef __MC6470_FUSE_HPP
#define __MC6470_FUSE_HPP

#include "mc6470_mag.hpp"
#include "mc6470_accel.hpp"
#include "mc6470_math.hpp"

namespace MC6470
{
    class FuseMC6470
    {
    public:
        FuseMC6470(const MagMC6470 &mag = MagMC6470(Wire), const AccMC6470 &accel = AccMC6470(1, Wire)) : the_mag(mag), the_acc(accel) {}

        std::auto_ptr<std::array<int32_t, 9>> get_orientation_matrix()
        {
            // std::array<int32_t, 9> c();
            int32_t *c = new int32_t[9];

            c[0] = east[0];
            c[1] = east[1];
            c[2] = east[2];

            c[3] = last_mag[0];
            c[4] = last_mag[1];
            c[5] = last_mag[2];

            c[6] = last_acc[0];
            c[7] = last_acc[1];
            c[8] = last_acc[2];

            std::array<int32_t, 9> *A = reinterpret_cast<std::array<int32_t, 9> *>(c);
            std::auto_ptr<std::array<int32_t, 9>> a_ptr(A);
            return a_ptr;
        }

        void required_grab_accel_callback(int16_t x, int16_t y, int16_t z)
        {
            last_acc[0] = x;
            last_acc[1] = y;
            last_acc[2] = z;
            update_fusion();
        }

        void required_grab_mag_callback(int16_t x, int16_t y, int16_t z)
        {
            last_mag[0] = x;
            last_mag[1] = y;
            last_mag[2] = z;
            update_fusion();
        }

        void required_grab_temp_callback(int8_t t)
        {
            temp_in_celcius = t;
        }

        bool display_fusion_update = false;

    private:
        const MagMC6470 &the_mag;
        const AccMC6470 &the_acc;

        int16_t last_acc[3];
        int16_t last_mag[3];
        int32_t east[3];
        int8_t temp_in_celcius;

        std::auto_ptr<std::array<int16_t, 3>> update_fusion()
        {
            auto cross = CrossProduct(last_acc, last_mag);
            east[0] = (*cross)[0];
            east[1] = (*cross)[1];
            east[2] = (*cross)[2];

            if (display_fusion_update)
            {
                auto orient = get_orientation_matrix();
                Serial.println("Got orient:");
                Serial.print("\t");
                Serial.print((*orient)[0]);
                Serial.print(", ");
                Serial.print((*orient)[1]);
                Serial.print(", ");
                Serial.println((*orient)[2]);
                Serial.print("\t");
                Serial.print((*orient)[3]);
                Serial.print(", ");
                Serial.print((*orient)[4]);
                Serial.print(", ");
                Serial.println((*orient)[5]);
                Serial.print("\t");
                Serial.print((*orient)[6]);
                Serial.print(", ");
                Serial.print((*orient)[7]);
                Serial.print(", ");
                Serial.println((*orient)[8]);
            }

            return cross;
        }
    };

}

#endif
// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "mc6470_settings.hpp"
#include <EEPROM.h>

namespace MC6470 {
MC6470Settings::~MC6470Settings() { del_mem_vector(); }

void MC6470Settings::add_mag_ardu_correction(std::array<std::array<float, 3>, 3>* mat, std::array<float, 3>* off)
{
    add_ardu_correction(mat, off, MEMTYPE::MAG_ARDU_CORRECTION);
}

void MC6470Settings::add_mag_ardu_correction(std::array<float, 12>* mat)
{
    mem.push_back(std::make_pair(MEMTYPE::MAG_ARDU_CORRECTION, reinterpret_cast<void*>(mat)));
}

void MC6470Settings::add_acc_ardu_correction(std::array<std::array<float, 3>, 3>* mat, std::array<float, 3>* off)
{
    add_ardu_correction(mat, off, MEMTYPE::ACC_ARDU_CORRECTION);
}

void MC6470Settings::add_acc_ardu_correction(std::array<float, 12>* mat)
{
    mem.push_back(std::make_pair(MEMTYPE::ACC_ARDU_CORRECTION, reinterpret_cast<void*>(mat)));
}

void MC6470Settings::add_imu_mag_correction(std::array<int16_t, 3>* off)
{
    mem.push_back(std::make_pair(MEMTYPE::MAG_IMU_CORRECTION, reinterpret_cast<void*>(off)));
}

void MC6470Settings::add_imu_acc_correction(std::array<float, 3>* scale, std::array<int16_t, 3>* off)
{
    std::array<uint8_t, 18>* full_mat = new std::array<uint8_t, 18>();
    memcpy(full_mat->data(), off->data(), 6);
    memcpy(full_mat->data() + 6, scale->data(), 12);
    mem.push_back(std::make_pair(MEMTYPE::ACC_IMU_CORRECTION, reinterpret_cast<void*>(full_mat)));
}

void MC6470Settings::add_imu_acc_correction(std::array<uint8_t, 18>* full)
{
    mem.push_back(std::make_pair(MEMTYPE::ACC_IMU_CORRECTION, reinterpret_cast<void*>(full)));
}

void MC6470Settings::add_acc_res_range(uint8_t res_range_setting)
{
    std::array<uint8_t, 1>* full_mat = new std::array<uint8_t, 1>({ res_range_setting });
    mem.push_back(std::make_pair(MEMTYPE::ACC_RES_RANGE, reinterpret_cast<void*>(full_mat)));
}

void MC6470Settings::add_acc_rate(uint8_t rate)
{
    std::array<uint8_t, 1>* full_mat = new std::array<uint8_t, 1>({ rate });
    mem.push_back(std::make_pair(MEMTYPE::ACC_RATE, reinterpret_cast<void*>(full_mat)));
}

int MC6470Settings::write_to_eeprom()
{
    int pos = start_address;
    for (auto i = mem.begin(); i != mem.end(); i++) {
        EEPROM.write(pos, (*i).first);
        pos += 1;
        switch ((*i).first) {
        case MEMTYPE::MAG_ARDU_CORRECTION:
        case MEMTYPE::ACC_ARDU_CORRECTION: {
            auto b = reinterpret_cast<std::array<uint8_t, 12 * 4>*>(((*i).second));
            for (auto j = b->begin(); j != b->end(); j++) {
                EEPROM.write(pos, (*j));
                pos += 1;
            }
            break;
        }

        case MEMTYPE::MAG_IMU_CORRECTION: {
            auto d = reinterpret_cast<std::array<uint8_t, 3 * 2>*>((*i).second);
            for (auto j = d->begin(); j != d->end(); j++) {
                EEPROM.write(pos, (*j));
                pos += 1;
            }
            break;
        }

        case MEMTYPE::ACC_IMU_CORRECTION: {
            auto f = reinterpret_cast<std::array<uint8_t, 18>*>((*i).second);
            for (auto j = f->begin(); j != f->end(); j++) {
                EEPROM.write(pos, (*j));
                pos += 1;
            }
            break;
        }

        case MEMTYPE::ACC_RES_RANGE:
        case MEMTYPE::ACC_RATE: {
            auto h = reinterpret_cast<std::array<uint8_t, 1>*>((*i).second);
            EEPROM.write(pos, (*h)[0]);
            pos += 1;
            break;
        }

        default:
            break;
        }
    }
    EEPROM.write(pos, MEMTYPE::END);
    pos++;
    return pos;
}

int MC6470Settings::read_from_eeprom()
{
    int pos = start_address;
    MEMTYPE current_section_type;
    do {
        current_section_type = static_cast<MEMTYPE>(EEPROM.read(pos));
        pos += 1;

        switch (current_section_type) {
            uint8_t bob[12 * 4];
            std::array<float, 12>* b;
            uint8_t ch;
        case MEMTYPE::MAG_ARDU_CORRECTION: {
            for (int i = 0; i < 12 * 4; i++) {
                bob[i] = EEPROM.read(pos);
                pos++;
            }
            b = reinterpret_cast<std::array<float, 12>*>(&bob);
            add_mag_ardu_correction(b);
            break;
        }
        case MEMTYPE::ACC_ARDU_CORRECTION: {
            for (int i = 0; i < 12 * 4; i++) {
                bob[i] = EEPROM.read(pos);
                pos++;
            }
            b = reinterpret_cast<std::array<float, 12>*>(&bob);
            add_acc_ardu_correction(b);
            break;
        }

        case MEMTYPE::MAG_IMU_CORRECTION: {
            uint8_t charles[3 * 2];
            for (int i = 0; i < 3 * 2; i++) {
                charles[i] = EEPROM.read(pos);
                pos++;
            }
            auto d = reinterpret_cast<std::array<int16_t, 3>*>(&charles);
            add_imu_mag_correction(d);
            break;
        }

        case MEMTYPE::ACC_IMU_CORRECTION: {
            uint8_t sam[18];
            for (int i = 0; i < 18; i++) {
                sam[i] = EEPROM.read(pos);
                pos++;
            }
            auto e = reinterpret_cast<std::array<uint8_t, 18>*>(&sam);
            add_imu_acc_correction(e);
            break;
        }

        case MEMTYPE::ACC_RES_RANGE: {
            ch = EEPROM.read(pos);
            pos++;
            add_acc_res_range(ch);
            break;
        }

        case MEMTYPE::ACC_RATE: {
            ch = EEPROM.read(pos);
            pos++;
            add_acc_rate(ch);
            break;
        }

        default:
            break;
        }
    } while (current_section_type != MEMTYPE::END);
    return pos;
}

void MC6470Settings::apply(MC6470::MagMC6470* mag_machine, MC6470::AccMC6470* acc_machine)
{
    MEMTYPE current_section_type;
    for (auto i = mem.begin(); i != mem.end(); i++) {
        current_section_type = (*i).first;
        switch (current_section_type) {
        case MEMTYPE::MAG_ARDU_CORRECTION: {
            auto b = static_cast<std::array<float, 12>*>((*i).second);
            std::array<std::array<float, 3>, 3> mat = { { 0 } };
            std::array<float, 3> off = { 0 };
            for (auto i = mat.begin(); i != mat.end(); i++) {
                for (auto j = (*i).begin(); j != (*i).end(); j++) {
                    auto ind_i = i - mat.begin();
                    auto ind_j = j - (*i).begin();
                    mat[ind_i][ind_j] = (*b)[ind_i * 4 + ind_j];
                }
            }
            for (auto j = off.begin(); j != off.end(); j++) {
                auto ind_j = j - off.begin();
                off[ind_j] = (*b)[ind_j * 4 + 3];
            }
            mag_machine->set_xform(mat);
            mag_machine->set_offset(off);
            break;
        }
        case MEMTYPE::ACC_ARDU_CORRECTION: {
            auto b = static_cast<std::array<float, 12>*>((*i).second);
            std::array<std::array<float, 3>, 3> mat = { { 0 } };
            std::array<float, 3> off = { 0 };
            for (auto i = mat.begin(); i != mat.end(); i++) {
                for (auto j = (*i).begin(); j != (*i).end(); j++) {
                    auto ind_i = i - mat.begin();
                    auto ind_j = j - (*i).begin();
                    mat[ind_i][ind_j] = (*b)[ind_i * 4 + ind_j];
                }
            }
            for (auto j = off.begin(); j != off.end(); j++) {
                auto ind_j = j - off.begin();
                off[ind_j] = (*b)[ind_j * 4 + 3];
            }
            mag_machine->set_xform(mat);
            mag_machine->set_offset(off);
            break;
        }

        case MEMTYPE::MAG_IMU_CORRECTION: {
            auto b = static_cast<std::array<int16_t, 3>*>((*i).second);
            mag_machine->set_mag_offset((*b)[0], (*b)[1], (*b)[2]);
            mag_machine->update_until_done();
            break;
        }

        case MEMTYPE::ACC_IMU_CORRECTION: {
            auto e = static_cast<std::array<uint8_t, 18>*>((*i).second);
            std::array<int16_t, 3> f({ 0 });
            std::array<float, 3> g({ 0 });
            memcpy(&f, e, 6);
            memcpy(&g, e + 6, 12);
            acc_machine->set_offset_and_gain(f[0], f[1], f[2], g[0], g[1], g[2]);
            acc_machine->update_until_done();
            break;
        }

        case MEMTYPE::ACC_RES_RANGE: {
            auto h = static_cast<std::array<uint8_t, 1>*>((*i).second);
            acc_machine->set_res_and_range((*h)[0], 0);
            acc_machine->update_until_done();
            break;
        }

        case MEMTYPE::ACC_RATE: {
            auto l = static_cast<std::array<uint8_t, 1>*>((*i).second);
            acc_machine->set_rate((*l)[0]);
            acc_machine->update_until_done();
            break;
        }

        default:
            break;
        }
    }
}

void MC6470Settings::add_ardu_correction(
    std::array<std::array<float, 3>, 3>* mat, std::array<float, 3>* off, MEMTYPE the_type)
{
    std::array<float, 12>* full_mat = new std::array<float, 12>();
    for (auto i = mat->begin(); i != mat->end(); i++) {
        for (auto j = (*i).begin(); j != (*i).end(); j++) {
            auto ind_i = i - mat->begin();
            auto ind_j = j - (*i).begin();
            (*full_mat)[ind_i * 4 + ind_j] = (*j);
        }
    }
    for (auto j = off->begin(); j != off->end(); j++) {
        auto ind_j = j - off->begin();
        (*full_mat)[ind_j * 4 + 3] = (*j);
    }
    mem.push_back(std::make_pair(the_type, reinterpret_cast<void*>(full_mat)));
}

void MC6470Settings::del_mem_vector()
{
    for (auto i = mem.begin(); i != mem.end(); i++) {
        switch ((*i).first) {
        case MEMTYPE::MAG_ARDU_CORRECTION:
        case MEMTYPE::ACC_ARDU_CORRECTION: {
            auto b = reinterpret_cast<std::array<uint8_t, 12 * 4>*>(((*i).second));
            delete b;
            break;
        }

        case MEMTYPE::MAG_IMU_CORRECTION: {
            auto d = reinterpret_cast<std::array<uint8_t, 3 * 2>*>((*i).second);
            delete d;
            break;
        }

        case MEMTYPE::ACC_IMU_CORRECTION: {
            auto f = reinterpret_cast<std::array<uint8_t, 18>*>((*i).second);
            delete f;
            break;
        }

        case MEMTYPE::ACC_RES_RANGE:
        case MEMTYPE::ACC_RATE: {
            auto h = reinterpret_cast<std::array<uint8_t, 1>*>((*i).second);
            delete h;
            break;
        }

        default:
            break;
        }
    }
    mem.clear();
}
} // namespace MC6470

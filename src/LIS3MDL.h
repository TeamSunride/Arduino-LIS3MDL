//
// Created by robosam2003 on 21/09/2022.
//

#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "protocol.h"
#include "Vector.h"
#include "LIS3MDL_registers.h"
#include "LIS3MDL_constants.h"


class LIS3MDL {
protected:
    protocol* device;
    double mag_conversion_factor;

public:
    LIS3MDL(TwoWire *pipe, uint32_t freq);

    LIS3MDL(byte chipSelect, SPIClass& spi, uint freq);

    void begin() {
        device->protocol_begin();
    }

    byte who_am_i();

    uint8_t enable_temp_sensor(bool enable);

    uint8_t set_mag_ODR(LIS3MDL_OUTPUT_DATA_RATES odr);

    // Self test enable

    uint8_t set_full_scale(LIS3MDL_FULL_SCALE full_scale);

    // REBOOT
    // SOFT_RST

    // LP
    // SIM
    uint8_t set_operating_mode(LIS3MDL_OPERATING_MODE mode);

    // BLE

    // FAST_READ
    // BDU

    Vector<double, 3> get_mag();





    void default_configuration();

};

#endif //LIS3MDL_H

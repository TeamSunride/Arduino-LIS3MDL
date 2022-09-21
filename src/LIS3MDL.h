//
// Created by robosam2003 on 21/09/2022.
//

#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "protocol.h"
#include "LIS3MDL_registers.h"
#include "LIS3MDL_constants.h"


class LIS3MDL {
protected:
    protocol* device;

public:
    LIS3MDL(TwoWire *pipe, uint32_t freq);

    LIS3MDL(byte chipSelect, SPIClass& spi, uint freq);


};

#endif //LIS3MDL_H

//
// Created by robosam2003 on 21/09/2022.
//
#include "LIS3MDL.h"

LIS3MDL::LIS3MDL(TwoWire *pipe, uint32_t freq) {
    device = new I2CProtocol(LIS3MDL_DEFAULT_I2C_ADDRESS, pipe, freq);

}

LIS3MDL::LIS3MDL(byte chipSelect, SPIClass &spi, uint freq) {
    // TODO: determine READ and WRITE bytes
    // TODO: determine SPI mode and bit order


}


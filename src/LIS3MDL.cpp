//
// Created by robosam2003 on 21/09/2022.
//
#include "LIS3MDL.h"

LIS3MDL::LIS3MDL(TwoWire *pipe, uint32_t freq) {
    device = new I2CProtocol(LIS3MDL_DEFAULT_I2C_ADDRESS, pipe, freq);
    mag_conversion_factor = 1/6842.0; // defaults to 4g full scale
    _reg = LIS3MDL_REGISTER();

}

LIS3MDL::LIS3MDL(byte chipSelect, SPIClass &spi, uint freq) {

    SPISettings settings(freq, MSBFIRST, SPI_MODE3);
    device = new SPIProtocol(chipSelect, spi, settings, READ_BYTE, WRITE_BYTE); /// Note: Auto increment is enabled by default.
    mag_conversion_factor = 1/6842.0; // defaults to 4g full scale
}

byte LIS3MDL::who_am_i() {
    return device->read_reg(LIS3MDL_REGISTER::LIS_WHO_AM_I);
}

uint8_t LIS3MDL::enable_temp_sensor(bool enable) {
    byte data = device->read_reg(LIS3MDL_REGISTER::LIS_CTRL_REG1);
    setBit(&data, 7, enable);
    return device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG1, data);
}

uint8_t LIS3MDL::set_mag_ODR(LIS3MDL_OUTPUT_DATA_RATES odr) {
    /// See Datasheet 7.2 or application note 2.2 for more info
    byte data = device->read_reg(LIS3MDL_REGISTER::LIS_CTRL_REG1);
    switch (odr) {
        case ODR_0_625_HZ ... ODR_80_HZ: { // ... is inclusive
            setBit(&data, 2, false); // disable fastODR
            data &= 0b11100011; // clear ODR bits
            data |= (odr << 2); // set ODR bits
            return device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG1, data);
        }
        case ODR_155_HZ: {
            setBit(&data, 2, true); // enable fastODR
            data &= 0b10011111; // clear OM bits
            data |= 0b01100000; // set OM bits to 11 for ultra-high performance mode
            uint8_t a = device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG1, data); // sets X and Y axes

            // now for Z axis
            byte ctrl4 = device->read_reg(LIS3MDL_REGISTER::LIS_CTRL_REG4);
            ctrl4 &= 0b10011111; // clear OM bits
            ctrl4 |= 0b01100000; // set OM bits to 11 for ultra-high performance mode
            uint8_t b = device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG4, ctrl4);
            return a && b; // zero for success
        }
        case ODR_300_HZ: {
            setBit(&data, 2, true); // enable fastODR
            data &= 0b10011111; // clear OM bits
            data |= 0b01000000; // set OM bits to 10 for high performance mode
            uint8_t a = device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG1, data); // sets X and Y axes

            // now for Z axis
            byte ctrl4 = device->read_reg(LIS3MDL_REGISTER::LIS_CTRL_REG4);
            ctrl4 &= 0b10011111; // clear OM bits
            ctrl4 |= 0b01000000; // set OM bits to 10 for high performance mode
            uint8_t b = device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG4, ctrl4);
            return a && b; // zero for success
        }
        case ODR_560_HZ: {
            setBit(&data, 2, true); // enable fastODR
            data &= 0b10011111; // clear OM bits
            data |= 0b00100000; // set OM bits to 01 for medium performance mode
            uint8_t a = device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG1, data); // sets X and Y axes

            // now for Z axis
            byte ctrl4 = device->read_reg(LIS3MDL_REGISTER::LIS_CTRL_REG4);
            ctrl4 &= 0b10011111; // clear OM bits
            ctrl4 |= 0b00100000; // set OM bits to 01 for medium performance mode
            uint8_t b = device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG4, ctrl4);
            return a && b; // zero for success
        }
        case ODR_1000_HZ: {
            setBit(&data, 2, true); // enable fastODR
            data &= 0b10011111; // clear OM bits
            data |= 0b00000000; // set OM bits to 00 for low performance mode
            uint8_t a = device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG1, data); // sets X and Y axes

            // now for Z axis
            byte ctrl4 = device->read_reg(LIS3MDL_REGISTER::LIS_CTRL_REG4);
            ctrl4 &= 0b10011111; // clear OM bits
            ctrl4 |= 0b00000000; // set OM bits to 00 for low performance mode
            uint8_t b = device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG4, ctrl4);
            return a && b; // zero for success
        }
    }
}

uint8_t LIS3MDL::set_full_scale(LIS3MDL_FULL_SCALE full_scale) {
    switch (full_scale) {
        case FS_4_GAUSS: {
            mag_conversion_factor = 1/6842.0;
            break;
        }
        case FS_8_GAUSS: {
            mag_conversion_factor = 1/3421.0;
            break;
        }
        case FS_12_GAUSS: {
            mag_conversion_factor = 1/2281.0;
            break;
        }
        case FS_16_GAUSS: {
            mag_conversion_factor = 1/1711.0;
            break;
        }

    }
    byte data = device->read_reg(LIS3MDL_REGISTER::LIS_CTRL_REG2);
    data &= 0b00001100; // clear FS bits and assert bits 7, 4, 1 and 0 are 0 (for correct operation of the device)
    data |= (full_scale << 5); // set FS bits
    return device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG2, data);
}

uint8_t LIS3MDL::set_operating_mode(LIS3MDL_OPERATING_MODE mode) {
    byte data = device->read_reg(LIS3MDL_REGISTER::LIS_CTRL_REG3);
    data &= 0b11111100; // clear MD bits
    data |= mode; // set MD bits
    return device->write_reg(LIS3MDL_REGISTER::LIS_CTRL_REG3, data);
}

Vector<double, 3> LIS3MDL::get_mag() {
    byte data[6];
    device->read_regs(LIS3MDL_REGISTER::LIS_OUT_X_L, data, 6);
    Vector<double, 3> mag;
    mag[0] = static_cast<int16_t>( (data[1] << 8) | data[0] ) * mag_conversion_factor;
    mag[1] = static_cast<int16_t>( (data[3] << 8) | data[2] ) * mag_conversion_factor;
    mag[2] = static_cast<int16_t>( (data[5] << 8) | data[4] ) * mag_conversion_factor;
    return mag;
}

void LIS3MDL::default_configuration() {
    set_mag_ODR(ODR_300_HZ);
    set_full_scale(FS_16_GAUSS);
    set_operating_mode(CONTINUOUS_CONVERSION);
}






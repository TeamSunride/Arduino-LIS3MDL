//
// Created by robos on 21/09/2022.
//

#ifndef LIS3MDL_CONSTANTS_H
#define LIS3MDL_CONSTANTS_H

#define WRITE_BYTE 0b01000000 /// Auto increment (bit 1) enabled by default.
#define READ_BYTE 0b11000000

#define LIS3MDL_DEFAULT_I2C_ADDRESS 0x1C

#define LIS_WHO_AM_I_ID 0x3D

enum LIS3MDL_OUTPUT_DATA_RATES {
    ODR_0_625_HZ =    0b000,
    ODR_1_25_HZ  =    0b001,
    ODR_2_5_HZ   =    0b010,
    ODR_5_HZ     =    0b011,
    ODR_10_HZ    =    0b100,
    ODR_20_HZ    =    0b101,
    ODR_40_HZ    =    0b110,
    ODR_80_HZ    =    0b111,
    ODR_155_HZ, // these are for the ultra-high performance mode
    ODR_300_HZ,
    ODR_560_HZ,
    ODR_1000_HZ
};

enum LIS3MDL_FULL_SCALE {
    FS_4_GAUSS = 0b00,
    FS_8_GAUSS = 0b01,
    FS_12_GAUSS = 0b10,
    FS_16_GAUSS = 0b11
};

enum LIS3MDL_OPERATING_MODE {
    CONTINUOUS_CONVERSION = 0b00,
    SINGLE_CONVERSION = 0b01, /// Single-conversion mode has to be used with sampling frequency from 0.625 Hz to 80Hz.
    POWER_DOWN = 0b10
};



#endif //LIS3MDL_CONSTANTS_H

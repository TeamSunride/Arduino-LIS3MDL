//
// Created by robos on 21/09/2022.
//

#ifndef LIS3MDL_CONSTANTS_H
#define LIS3MDL_CONSTANTS_H


#define LIS3MDL_DEFAULT_I2C_ADDRESS 0x1C


namespace LIS3MDL {

    const uint8_t WHO_AM_I_ID = 0x3D;
    const uint8_t WRITE_BYTE = 0b01000000; /// Auto increment (bit 1) enabled by default.
    const uint8_t READ_BYTE = 0b11000000;


    enum OUTPUT_DATA_RATES {
        ODR_0_625_HZ = 0b000,
        ODR_1_25_HZ = 0b001,
        ODR_2_5_HZ = 0b010,
        ODR_5_HZ = 0b011,
        ODR_10_HZ = 0b100,
        ODR_20_HZ = 0b101,
        ODR_40_HZ = 0b110,
        ODR_80_HZ = 0b111,
        ODR_155_HZ, // these are for the ultra-high performance mode
        ODR_300_HZ,
        ODR_560_HZ,
        ODR_1000_HZ
    };

    enum FULL_SCALE {
        FS_4_GAUSS = 0b00,
        FS_8_GAUSS = 0b01,
        FS_12_GAUSS = 0b10,
        FS_16_GAUSS = 0b11
    };

    enum AXIS_OPERATING_MODE {
        LOW_POWER = 0b00,
        MEDIUM_PERFORMANCE = 0b01,
        HIGH_PERFORMANCE = 0b10,
        ULTRA_HIGH_PERFORMANCE = 0b11
    };


    enum SYSTEM_OPERATING_MODE {
        CONTINUOUS_CONVERSION = 0b00,
        SINGLE_CONVERSION = 0b01, /// Single-conversion mode has to be used with sampling frequency from 0.625 Hz to 80Hz.
        POWER_DOWN = 0b10
    };

    enum AXIS {
        X_AXIS = 2,
        Y_AXIS = 1,
        Z_AXIS = 0
    };

    enum INTERRUPTS {
        PTH_X = 7,
        PTH_Y = 6,
        PTH_Z = 5,
        NTH_X = 4,
        NTH_Y = 3,
        NTH_Z = 2,
        MROI = 1
                //        INT =  0b00000001
    };

}

#endif //LIS3MDL_CONSTANTS_H


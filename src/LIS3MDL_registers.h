//
// Created by robos on 21/09/2022.
//

#ifndef LIS3MDL_REGISTERS_H
#define LIS3MDL_REGISTERS_H


namespace LIS3MDL {

    enum REGISTER { // Registers are scoped to avoid confusion with other sensor registers names - Please use the scoping when you can
        // Reserved: 00-0E
        WHO_AM_I = 0x0F, // R
        // Reserved: 10-1F
        CTRL_REG1 = 0x20, // R/W
        CTRL_REG2 = 0x21, // R/W
        CTRL_REG3 = 0x22, // R/W
        CTRL_REG4 = 0x23, // R/W
        CTRL_REG5 = 0x24, // R/W
        // Reserved: 25-26
        STATUS_REG = 0x27, // R
        OUT_X_L = 0x28, // R
        OUT_X_H = 0x29, // R
        OUT_Y_L = 0x2A, // R
        OUT_Y_H = 0x2B, // R
        OUT_Z_L = 0x2C, // R
        OUT_Z_H = 0x2D, // R
        TEMP_OUT_L = 0x2E, // R
        TEMP_OUT_H = 0x2F, // R
        INT_CFG = 0x30, // R/W
        INT_SRC = 0x31, // R
        INT_THS_L = 0x32, // R/W
        INT_THS_H = 0x33, // R/W

    };

}

#endif //LIS3MDL_REGISTERS_H

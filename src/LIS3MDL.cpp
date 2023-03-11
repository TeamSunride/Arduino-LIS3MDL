//
// Created by robosam2003 on 21/09/2022.
//
#include "LIS3MDL.h"

namespace LIS3MDL {

    LIS3MDL::LIS3MDL(TwoWire *pipe, uint32_t freq) {
        device = new I2CProtocol(LIS3MDL_DEFAULT_I2C_ADDRESS, pipe, freq);
        mag_conversion_factor = 1 / 6842.0; // defaults to 4g full scale

    }

    LIS3MDL::LIS3MDL(byte chipSelect, SPIClass &spi, uint freq) {

        SPISettings settings(freq, MSBFIRST, SPI_MODE3);
        device = new SPIProtocol(chipSelect, spi, settings, READ_BYTE,
                                 WRITE_BYTE); /// Note: Auto increment is enabled by default.
        mag_conversion_factor = 1 / 6842.0; // defaults to 4g full scale
    }

    byte LIS3MDL::who_am_i() {
        return device->read_reg(REGISTER::WHO_AM_I);
    }

    uint8_t LIS3MDL::enable_temp_sensor(bool enable) {
        byte data = device->read_reg(REGISTER::CTRL_REG1);
        setBit(&data, 7, enable);
        return device->write_reg(REGISTER::CTRL_REG1, data);
    }

    uint8_t LIS3MDL::set_mag_ODR(OUTPUT_DATA_RATES odr) {
        /// See Datasheet 7.2 or application note 2.2 for more info
        byte data = device->read_reg(REGISTER::CTRL_REG1);
        switch (odr) {
            case ODR_0_625_HZ ... ODR_80_HZ: { // ... is inclusive
                setBit(&data, 1, false); // disable fastODR
                data &= 0b11100011; // clear ODR bits
                data |= (odr << 2); // set ODR bits
                return device->write_reg(REGISTER::CTRL_REG1, data);
            }
            case ODR_155_HZ: {
                setBit(&data, 1, true); // enable fastODR
                data &= 0b10011111; // clear OM bits
                data |= (AXIS_OPERATING_MODE::ULTRA_HIGH_PERFORMANCE
                        << 5); // set OM bits to 11 for ultra-high performance mode
//                data |= 0b01100000; // set OM bits to 11 for ultra-high performance mode
                uint8_t a = device->write_reg(REGISTER::CTRL_REG1, data); // sets X and Y axes

                // now for Z axis
                byte ctrl4 = device->read_reg(REGISTER::CTRL_REG4);
                ctrl4 &= 0b11110011; // clear OM bits
                ctrl4 |= (AXIS_OPERATING_MODE::ULTRA_HIGH_PERFORMANCE
                        << 2); // set OM bits to 11 for ultra-high performance mode
//                ctrl4 &= 0b11110011; // set OM bits to 11 for ultra-high performance mode
                uint8_t b = device->write_reg(REGISTER::CTRL_REG4, ctrl4);
                return a && b; // zero for success
            }
            case ODR_300_HZ: {
                setBit(&data, 1, true); // enable fastODR
                data &= 0b10011111; // clear OM bits
                data |= (AXIS_OPERATING_MODE::HIGH_PERFORMANCE << 5); // set OM bits to 10 for high performance mode
//                data |= 0b01000000; // set OM bits to 10 for high performance mode
                uint8_t a = device->write_reg(REGISTER::CTRL_REG1, data); // sets X and Y axes

                // now for Z axis
                byte ctrl4 = device->read_reg(REGISTER::CTRL_REG4);
                ctrl4 &= 0b11110011; // clear OM bits
                ctrl4 |= (AXIS_OPERATING_MODE::HIGH_PERFORMANCE << 2); // set OM bits to 10 for high performance mode
//                ctrl4 |= 0b01000000; // set OM bits to 10 for high performance mode
                uint8_t b = device->write_reg(REGISTER::CTRL_REG4, ctrl4);
                return a && b; // zero for success
            }
            case ODR_560_HZ: {
                setBit(&data, 1, true); // enable fastODR
                data &= 0b10011111; // clear OM bits
                data |= (AXIS_OPERATING_MODE::MEDIUM_PERFORMANCE << 5); // set OM bits to 01 for medium performance mode
//                data |= 0b00100000; // set OM bits to 01 for medium performance mode
                uint8_t a = device->write_reg(REGISTER::CTRL_REG1, data); // sets X and Y axes

                // now for Z axis
                byte ctrl4 = device->read_reg(REGISTER::CTRL_REG4);
                ctrl4 &= 0b11110011; // clear OM bits
                ctrl4 |= (AXIS_OPERATING_MODE::MEDIUM_PERFORMANCE
                        << 2); // set OM bits to 01 for medium performance mode
//                ctrl4 |= 0b00100000; // set OM bits to 01 for medium performance mode
                uint8_t b = device->write_reg(REGISTER::CTRL_REG4, ctrl4);
                return a && b; // zero for success
            }
            case ODR_1000_HZ: {
                setBit(&data, 1, true); // enable fastODR
                data &= 0b10011111; // clear OM bits
                data |= (AXIS_OPERATING_MODE::LOW_POWER << 5); // set OM bits to 00 for low power mode
//                data |= 0b00000000; // set OM bits to 00 for low power mode
                uint8_t a = device->write_reg(REGISTER::CTRL_REG1, data); // sets X and Y axes

                // now for Z axis
                byte ctrl4 = device->read_reg(REGISTER::CTRL_REG4);
                ctrl4 &= 0b11110011; // clear OM bits
                ctrl4 |= (AXIS_OPERATING_MODE::LOW_POWER << 2); // set OM bits to 00 for low power mode
//                ctrl4 |= 0b00000000; // set OM bits to 00 for low power mode
                uint8_t b = device->write_reg(REGISTER::CTRL_REG4, ctrl4);
                return a && b; // zero for success
            }
        }
        return -1;
    }

    uint8_t LIS3MDL::perform_self_test() {
        /// NOTE: THIS CHANGES THE ODRS AND FULL SCALES OF THE MAGNETOMETER
        // TODO: add a way to restore the previous settings


        Serial.println("Self test procedure, please keep the device still");
        // Init sensor, FS = 12 Gauss,  Continuous-Measurement mode, ODR = 80 Hz
        device->write_reg(REGISTER::CTRL_REG1, 0X1C);
        device->write_reg(REGISTER::CTRL_REG2, 0X40);
        delay(20); // wait 20ms
        device->write_reg(REGISTER::CTRL_REG3, 0X00);

        delay(20);
        // Power up, check the ZYXDA bit in the STATUS_REG register
        while(!get_mag_drdy_status());
        Vector<double , 3> discard = get_mag(); // read the OUT regs and discard the data

        // Average 5 samples
        Vector<double , 3> average_NOST = {0, 0, 0};
        for (int i = 0; i < 20; i++) {
            while(!get_mag_drdy_status());
            Vector<double, 3> raw = get_mag();
            average_NOST += raw; // sum the values
            delay(20);
        }
        average_NOST /= 20; // divide by 5 to get the average

        // Enable self test
        device->write_reg(REGISTER::CTRL_REG1, 0X1D);
        delay(60); // wait 60ms

        // Power up, check the ZYXDA bit in the STATUS_REG register
        while(!get_mag_drdy_status());
        Vector<double , 3> discard2 = get_mag(); // read the OUT regs and discard the data


        Vector<double, 3> average_ST = {0, 0, 0};
        // Take 5 samples
        Vector<double, 3> raw;
        for (int i = 0; i < 20; i++) {
            while(!get_mag_drdy_status());
            raw = get_mag();
            average_ST += raw; // sum the values

            delay(20);
        }
        average_ST /= 20; // divide by 5 to get the average

        // Disable self test
        device->write_reg(REGISTER::CTRL_REG1, 0X1C);

        // Print out all the min, max, average and raw values
        Serial.println("Average_NOST, Average_ST, Raw");
        for (int i = 0; i < 3; i++) {
            Serial.print(average_NOST[i]);
            Serial.print(", ");
            Serial.print(average_ST[i]);
            Serial.print(", ");
            Serial.println(raw[i]);
        }

        // Calculate if the self test passed
//        for (int i = 0; i < 3; i++) {
//            pass &= (abs(minST[i]) <= (raw[i] - average_NOST[i])) &&
//                    ((raw[i] - average_NOST[i]) <= abs(maxST[i]));
//        }
        if (    (abs(1.0) <= abs(average_ST[0] - average_NOST[0])) && (abs(average_ST[0] - average_NOST[0]) <= abs(3.0)) && // limits fpr
                (abs(1.0) <= abs(average_ST[1] - average_NOST[1])) && (abs(average_ST[1] - average_NOST[1]) <= abs(3.0)) &&
                (abs(0.1) <= abs(average_ST[2] - average_NOST[2])) && (abs(average_ST[2] - average_NOST[2]) <= abs(1.0))
                ) {
            return true;
        }
        else {
            return false;
        }

    }

    uint8_t LIS3MDL::set_full_scale(FULL_SCALE full_scale) {
        switch (full_scale) {
            case FS_4_GAUSS: {
                mag_conversion_factor = 1 / 6842.0;
                break;
            }
            case FS_8_GAUSS: {
                mag_conversion_factor = 1 / 3421.0;
                break;
            }
            case FS_12_GAUSS: {
                mag_conversion_factor = 1 / 2281.0;
                break;
            }
            case FS_16_GAUSS: {
                mag_conversion_factor = 1 / 1711.0;
                break;
            }
        }
        byte data = device->read_reg(REGISTER::CTRL_REG2);
        data &= 0b00001100; // clear FS bits and assert bits 7, 4, 1 and 0 are 0 (for correct operation of the device)
        data |= (full_scale << 5); // set FS bits
        return device->write_reg(REGISTER::CTRL_REG2, data);
    }

    uint8_t LIS3MDL::reboot_memory_content() {
        byte data = device->read_reg(REGISTER::CTRL_REG2);
        setBit(&data, 3, true); // set reboot bit
        return device->write_reg(REGISTER::CTRL_REG2, data);
    }

    uint8_t LIS3MDL::soft_reset() {
        byte data = device->read_reg(REGISTER::CTRL_REG2);
        setBit(&data, 2, true); // set soft reset bit
        return device->write_reg(REGISTER::CTRL_REG2, data);
    }

    uint8_t LIS3MDL::enter_low_power_mode(bool enable) {
        byte data = device->read_reg(REGISTER::CTRL_REG3);
        setBit(&data, 5, enable); // set LP bit
        return device->write_reg(REGISTER::CTRL_REG3, data);
    }

    uint8_t LIS3MDL::set_spi_mode_3_wire(bool enable) {
        byte data = device->read_reg(REGISTER::CTRL_REG3);
        setBit(&data, 2, enable); // set SIM bit
        return device->write_reg(REGISTER::CTRL_REG3, data);
    }

    uint8_t LIS3MDL::set_operating_mode(SYSTEM_OPERATING_MODE mode) {
        byte data = device->read_reg(REGISTER::CTRL_REG3);
        data &= 0b11111100; // clear MD bits
        data |= mode; // set MD bits
        return device->write_reg(REGISTER::CTRL_REG3, data);
    }

    uint8_t LIS3MDL::set_little_endian(bool enable) {
        byte data = device->read_reg(REGISTER::CTRL_REG4);
        setBit(&data, 1, enable);
        return device->write_reg(REGISTER::CTRL_REG4, data);
    }

    uint8_t LIS3MDL::set_fast_read(bool enable) {
        byte data = device->read_reg(REGISTER::CTRL_REG5);
        setBit(&data, 7, enable);
        return device->write_reg(REGISTER::CTRL_REG5, data);
    }

    uint8_t LIS3MDL::block_output_data_update(bool enable) {
        byte data = device->read_reg(REGISTER::CTRL_REG5);
        setBit(&data, 6, enable);
        return device->write_reg(REGISTER::CTRL_REG5, data);
    }

    Vector<int16_t, 3> LIS3MDL::get_raw_mag() {
        byte data[6];
        device->read_regs(REGISTER::OUT_X_L, data, 6);
        Vector<int16_t, 3> raw_mag;
        // defaults to big endian, so we are using big endian.
        raw_mag[0] = (int16_t) ((data[1] << 8) | data[0]);
        raw_mag[1] = (int16_t) ((data[3] << 8) | data[2]);
        raw_mag[2] = (int16_t) ((data[5] << 8) | data[4]);
        return raw_mag;
    }

    Vector<double, 3> LIS3MDL::convert_raw_mag_to_gauss(Vector<int16_t, 3> raw_mag) {
        Vector<double, 3> mag;
        mag[0] = raw_mag[0] * mag_conversion_factor;
        mag[1] = raw_mag[1] * mag_conversion_factor;
        mag[2] = raw_mag[2] * mag_conversion_factor;
        return mag;
    }

    Vector<double, 3> LIS3MDL::get_mag() {
        byte data[6];
        device->read_regs(REGISTER::OUT_X_L, data, 6);
        Vector<double, 3> mag;
        // defaults to big endian, so we are using big endian.
        mag[0] = static_cast<int16_t>((data[1] << 8) | data[0] ) * mag_conversion_factor;
        mag[1] = static_cast<int16_t>((data[3] << 8) | data[2] ) * mag_conversion_factor;
        mag[2] = static_cast<int16_t>((data[5] << 8) | data[4] ) * mag_conversion_factor;
        return mag;
    }

    double LIS3MDL::get_temp_celsius() {
        byte data[2];
        device->read_regs(REGISTER::TEMP_OUT_L, data, 2);
        return static_cast<int16_t>((data[1] << 8) | data[0]) / 8.0 + 25.0;
    }

    uint8_t LIS3MDL::get_mag_drdy_status() {
        byte data = device->read_reg(REGISTER::STATUS_REG);
        return getBit(data, 3); /// get ZYXDA bit
    }

    uint8_t LIS3MDL::set_interrupt_on_axis(AXIS axis, bool enable) {
        byte data = device->read_reg(REGISTER::INT_CFG);
        data |= (enable << (6 + axis)); // set XIE, YIE, or ZIE bit
        return device->write_reg(REGISTER::INT_CFG, data);
    }

    uint8_t LIS3MDL::set_interrupt_active_high(bool enable) {
        byte data = device->read_reg(REGISTER::INT_CFG);
        setBit(&data, 2, enable);
        return device->write_reg(REGISTER::INT_CFG, data);
    }

    uint8_t LIS3MDL::set_interrupt_threshold(double threshold) {
        int16_t threshold_raw = threshold / mag_conversion_factor;
        byte data[2];
        data[0] = threshold_raw & 0xFF;
        data[1] = (threshold_raw >> 8) & 0xFF;
        return device->write_regs(REGISTER::INT_THS_L, data, 2);
    }

    uint8_t LIS3MDL::enable_interrupt(bool enable) {
        byte data = device->read_reg(REGISTER::INT_CFG);
        setBit(&data, 0, enable);
        return device->write_reg(REGISTER::INT_CFG, data);
    }

    void LIS3MDL::interrupt_service_routine() {
        Serial.println("AN INTERRUPT HAS OCCURRED");
    }




    void LIS3MDL::default_configuration() {
        enable_temp_sensor(true);
        set_interrupt_active_high(true);
        block_output_data_update(true);
        enable_interrupt(true);

        set_mag_ODR(ODR_300_HZ);
        set_full_scale(FS_16_GAUSS);
        set_operating_mode(CONTINUOUS_CONVERSION);
    }

}




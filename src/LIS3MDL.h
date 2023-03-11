//
// Created by robosam2003 on 21/09/2022.
//

#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "protocol.h"
#include "Vector.h"
#include "LIS3MDL_registers.h"
#include "LIS3MDL_constants.h"

namespace LIS3MDL {

    class LIS3MDL {
    protected:
        protocol *device;
        double mag_conversion_factor;
//        REGISTER _reg;

    public:
        LIS3MDL(TwoWire *pipe, uint32_t freq);

        LIS3MDL(byte chipSelect, SPIClass &spi, uint freq);

        void begin() {
            device->protocol_begin();
        }

        /**
         * @brief Get the device ID.
         * @return Device ID
         */
        byte who_am_i();

        /**
         * @brief Enable or disable the temperature sensor.
         * @param enable
         * @return Status Code (0 for success)
         */
        uint8_t enable_temp_sensor(bool enable);

        /**
         * @brief Set the output data rate for the magnetometer.
         * @param odr
         * @return Status Code (0 for success)
         */
        uint8_t set_mag_ODR(OUTPUT_DATA_RATES odr);

        // Self test enable
        uint8_t perform_self_test();

        /**
         * @brief Set the full scale for the magnetometer.
         * @param full_scale
         * @return Status Code (0 for success)
         */
        uint8_t set_full_scale(FULL_SCALE full_scale);

        /**
         * @brief Reboot memory content.
         * @return Status Code (0 for success)
         */
        uint8_t reboot_memory_content();

        /**
         * @brief Configuration registers and user register reset function.
         * @return Status Code (0 for success)
         */
        uint8_t soft_reset();

        /**
         *  Low-power mode configuration. Default value: 0
         *  If this bit is ‘1’, DO[2:0] is set to 0.625 Hz and the system performs, for each
         *  channel, the minimum number of averages. Once the bit is set to ‘0’, the magnetic data rate is configured by the DO bits in CTRL_REG1 (20h) register.
         * @brief Enable or disable low power mode - Note: this sets the ODR to 0.625 Hz
         * @param enable
         * @return Status Code (0 for success)
         */
        uint8_t enter_low_power_mode(bool enable);


        /**
         * @brief Set the SPI mode to 3 wire.
         * @param enable
         * @return Status Code (0 for success)
         */
        uint8_t set_spi_mode_3_wire(bool enable);


        /**
         * @brief Set the system operating mode.
         * @param mode
         * @return Status Code (0 for success)
         */
        uint8_t set_operating_mode(SYSTEM_OPERATING_MODE mode);

        /**
         * @brief Set the output to little endian format.
         * @param enable
         * @return Status Code (0 for success)
         */
        uint8_t set_little_endian(bool enable);

        /**
         * @brief Set the fast read mode. - FAST READ allows reading the high part of DATA OUT only in order to increase reading efficiency
         * @param enable
         * @return Status Code (0 for success)
         */
        uint8_t set_fast_read(bool enable);

        /**
         * @brief Set the block data update mode to true - Block data update for magnetic data. Default value: 0
                    (0: continuous update;
                    1: output registers not updated until MSb and LSb have been read)
         * @param enable
         * @return Status Code (0 for success)
         */
        uint8_t block_output_data_update(bool enable);

        /**
         * @brief get the raw magnetometer data as an int16_t vector.
         * @return The vector of Raw Magnetometer Data.
         */
        Vector<int16_t, 3> get_raw_mag();

        /**
         * @brief Convert the raw magnetometer data to gauss.
         * @param raw_mag
         * @return The vector of converted magnetometer data.
         */
        Vector<double, 3> convert_raw_mag_to_gauss(Vector<int16_t, 3> raw_mag);

        /**
         * @brief Get the magnetometer data in gauss.
         * @return The vector of magnetometer data.
         */
        Vector<double, 3> get_mag();

        /**
         * @brief Get the raw temperature data in Celcius.
         * @return The raw temperature data in Celcius
         */
        double get_temp_celsius();


        /**
         * @brief Get the ZYXDA bit status of the magnetometer data ready flag
         * @return The status of the magnetometer data ready flag (1 for ready, 0 for not ready)
         */
        uint8_t get_mag_drdy_status();

        /**
         * @brief Set the interrupt on an axis
         * @param axis
         * @param enable
         * @return Status Code (0 for success)
         */
        uint8_t set_interrupt_on_axis(AXIS axis, bool enable);

        /**
         * @brief Set the interrupt active high or low
         * @param enable
         * @return Status Code (0 for success)
         */
        uint8_t set_interrupt_active_high(bool enable);

        /**
         * @brief Set the interrupt threshold in gauss
         * @param threshold
         * @return Status Code (0 for success)
         */
        uint8_t set_interrupt_threshold(double threshold);

        /**
         * @brief enable or disable the interrupt
         * @param enable
         * @return Status Code (0 for success)
         */
        uint8_t enable_interrupt(bool enable);

        /**
         * @brief A predefined interrupt service routine.
         */
        static void interrupt_service_routine();

        /**
         * @brief A default configuration for the magnetometer - use always call this in your setup.
         */
        void default_configuration();

    };

}


#endif //LIS3MDL_H


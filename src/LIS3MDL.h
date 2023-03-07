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

        byte who_am_i();

        uint8_t enable_temp_sensor(bool enable);

        uint8_t set_mag_ODR(OUTPUT_DATA_RATES odr);

        // Self test enable

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

        uint8_t set_operating_mode(SYSTEM_OPERATING_MODE mode);

        uint8_t set_little_endian(bool enable);

        // FAST_READ
        // BDU

        Vector<int16_t, 3> get_raw_mag();

        Vector<double, 3> convert_raw_mag_to_gauss(Vector<int16_t, 3> raw_mag);

        Vector<double, 3> get_mag();

        double get_temp_celsius();

        uint8_t get_mag_drdy_status();

        uint8_t get_temp_drdy_status();



        void default_configuration();

    };

}


#endif //LIS3MDL_H


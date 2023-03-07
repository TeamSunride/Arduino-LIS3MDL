#include <Arduino.h>
#include "LIS3MDL.h"


#define LIS_CS 37
#define CS_LSM 40
LIS3MDL::LIS3MDL LIS(LIS_CS, SPI, 4000000);



void setup() {
    // Pull other devices on the bus high.
    pinMode(CS_LSM, OUTPUT);
    digitalWrite(CS_LSM, HIGH);

    LIS.begin();
    LIS.default_configuration();
    LIS.set_mag_ODR(LIS3MDL::OUTPUT_DATA_RATES::ODR_1000_HZ);
}

void loop() {
    Vector<int16_t, 3> mag_i = LIS.get_raw_mag();
    Vector<double, 3> mag = LIS.convert_raw_mag_to_gauss(mag_i);
    Serial.printf("MAG: %lf, %lf, %lf\n", mag[0], mag[1], mag[2]);
//    Serial.printf("TEMP: %lf\n", LIS.get_temp_celsius());
    delay(1);
}
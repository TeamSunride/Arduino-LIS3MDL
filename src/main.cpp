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
    Vector<double, 3> mag = LIS.get_mag();
    Serial.printf("MAG: %lf, %lf, %lf\n", mag[0], mag[1], mag[2]);
    delay(1);
}
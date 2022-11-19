#include <Arduino.h>
#include "LIS3MDL.h"


#define CS 37
LIS3MDL LIS(CS, SPI, 4000000);


void setup() {
    LIS.begin();
    LIS.default_configuration();

}

void loop() {
    Vector<double, 3> mag = LIS.get_mag();
    Serial.printf("MAG: %lf, %lf, %lf\n", mag[0], mag[1], mag[2]);
    delay(10);
}
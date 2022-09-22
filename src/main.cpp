#include <Arduino.h>
#include "LIS3MDL.h"


LIS3MDL LIS(&Wire, 1000000); // i2c protocol constructor


void setup() {
    LIS.begin();
    LIS.default_configuration();
}

void loop() {
    Vector<double, 3> mag = LIS.get_mag();
    Serial.printf("MAG: %lf, %lf, %lf\n", mag[0], mag[1], mag[2]);
    delay(10);
}
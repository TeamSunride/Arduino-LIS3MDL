#include <Arduino.h>
#include "LIS3MDL.h"


LIS3MDL LIS(&Wire, 1000000); // i2c protocol constructor


void setup() {
    LIS.begin();
}

void loop() {
    byte who = LIS.who_am_i();
    Serial.printf("WHO: %X\n", who);


    delay(10);
}
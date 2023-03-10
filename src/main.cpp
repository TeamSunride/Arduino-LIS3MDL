#include <Arduino.h>
#include "LIS3MDL.h"


#define LIS_CS 10
#define CS_LSM 40
#define LIS_INT 32
//LIS3MDL::LIS3MDL LIS(LIS_CS, SPI, 4000000);
LIS3MDL::LIS3MDL LIS(&Wire, 100000);


void setup() {
//    // Pull other devices on the bus high.
//    pinMode(CS_LSM, OUTPUT);
//    digitalWrite(CS_LSM, HIGH);

    LIS.begin();
    LIS.default_configuration();
    LIS.set_mag_ODR(LIS3MDL::OUTPUT_DATA_RATES::ODR_155_HZ);
    LIS.set_interrupt_on_axis(LIS3MDL::AXIS::X_AXIS, true);
    LIS.set_interrupt_on_axis(LIS3MDL::AXIS::Y_AXIS, true);
    LIS.set_interrupt_on_axis(LIS3MDL::AXIS::Z_AXIS, true);

    LIS.set_interrupt_threshold(3);

    attachInterrupt(digitalPinToInterrupt(LIS_INT), LIS.interrupt_service_routine, CHANGE);
}

void loop() {
//    Vector<int16_t, 3> mag_i = LIS.get_raw_mag();
//    Vector<double, 3> mag = LIS.convert_raw_mag_to_gauss(mag_i);
    Vector<double, 3> mag = LIS.get_mag();
//    mag.normalize();
    Serial.printf("MAG: %lf, %lf, %lf\r\n", mag[0], mag[1], mag[2]);
    int interrupt = digitalRead(LIS_INT);
    Serial.println(interrupt);
//    Serial.printf("0, %lf, %lf, %lf\r\n", mag[0], mag[1], mag[2]); // For quaternion visualizer on MATLAB
//    Serial.printf("TEMP: %lf\n", LIS.get_temp_celsius());
    delay(100);
//    byte who_am_i = LIS.who_am_i();
//    Serial.println(who_am_i);
//    delay(100);
}
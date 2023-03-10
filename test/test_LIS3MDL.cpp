//
// Created by robos on 08/03/2023.
//
#include <unity.h>
#include "LIS3MDL.h"

#define LIS_CS 37
#define CS_LSM 40
#define LIS_INT 39
LIS3MDL::LIS3MDL LISTest(LIS_CS, SPI, 4000000);

void setUp(void) {
    // pull CS of other devices on the bus high
    pinMode(CS_LSM, OUTPUT);
    digitalWrite(CS_LSM, HIGH);

    Serial.begin(115200);
    LISTest.begin();
    LISTest.default_configuration();
    LISTest.set_mag_ODR(LIS3MDL::OUTPUT_DATA_RATES::ODR_1000_HZ);
}

void tearDown(void) {
    // clean stuff up here
}

/* ----------------------------------Tests---------------------------------------- */
void test_who_am_i(void) {
    TEST_ASSERT_EQUAL(LISTest.who_am_i(), LIS3MDL::WHO_AM_I_ID);
}

void test_get_mag(void) {
    Vector<double, 3> mag = LISTest.get_mag();
    TEST_ASSERT(mag.norm() != 0);
}

void test_get_temp_celsius(void) {
    double temp = LISTest.get_temp_celsius();
    TEST_ASSERT(temp != 0);
}

void test_get_raw_mag(void) {
    // Set ODR to low so that the two reads are probably the same.
    LISTest.set_mag_ODR(LIS3MDL::OUTPUT_DATA_RATES::ODR_155_HZ);
    // read twice in quick succession
    Vector<double, 3> mag_d = LISTest.get_mag();
    Vector<int16_t, 3> mag = LISTest.get_raw_mag();
    Vector<double, 3> mag_conv = LISTest.convert_raw_mag_to_gauss(mag);
    TEST_ASSERT_EQUAL(mag_d.norm(), mag_conv.norm());

    // Set the ODR back to 1000 Hz
    LISTest.set_mag_ODR(LIS3MDL::OUTPUT_DATA_RATES::ODR_1000_HZ);

}

// Self test
void test_self_test(void) {
    TEST_ASSERT_EQUAL(true, LISTest.perform_self_test());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_who_am_i);
    RUN_TEST(test_get_mag);
    RUN_TEST(test_get_temp_celsius);
    RUN_TEST(test_get_raw_mag);
    RUN_TEST(test_self_test);
    UNITY_END();

    return 0;
}


#include "unity.h"
#include <Arduino.h>

void setUp(void) {}

void tearDown(void) {}

void test_function_1(void) {}

void test_function_2(void) {}

void setup() {
  delay(2000);

  UNITY_BEGIN();
  
  RUN_TEST(test_function_1);
  RUN_TEST(test_function_2);
  
  UNITY_END();
}

void loop() {}
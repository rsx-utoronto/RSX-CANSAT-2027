#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  static uint32_t n = 0;
  Serial.printf("packet %lu\n", n++);
  delay(250);       // 4 Hz, same rate as our real telemetry
}
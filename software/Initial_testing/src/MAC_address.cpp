#include <Arduino.h>
#include <WiFi.h>
void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_STA);     // must come before macAddress()
  Serial.println(WiFi.macAddress());
}
void loop() {}
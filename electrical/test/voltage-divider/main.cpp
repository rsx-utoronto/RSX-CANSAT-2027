// reads voltage on solar panel
#include <Arduino.h>  //gives access to arduino functions

const int solar1_raw = 36;  // pin 4 = GPIO36
const int solar2_raw = 39;  // pin 5 = GPIO39
const float R_top = 13000.0;
const float R_bot = 10000.0;
const float div_decode = (R_top + R_bot) / R_bot;

void setup() {
  Serial.begin(115200);  // communicate at 115200 baud

  analogReadResolution(12);                       // sets ADC to 12 bits
  analogSetPinAttenuation(solar1_raw, ADC_11db);  // use 11dB attenuation
  analogSetPinAttenuation(solar2_raw, ADC_11db);
}

// reads and prints voltage on the solar panels every 500ms
void loop() {
  float pin36_mV = analogReadMilliVolts(solar1_raw);
  float pin39_mV = analogReadMilliVolts(solar2_raw);
  float solar1_V = (pin36_mV / 1000.0) * div_decode;
  float solar2_V = (pin39_mV / 1000.0) * div_decode;

  Serial.print("Solar1: ");
  Serial.println(solar1_V, 3);
  Serial.print("Solar2: ");
  Serial.println(solar2_V, 3);

  delay(500);
}
